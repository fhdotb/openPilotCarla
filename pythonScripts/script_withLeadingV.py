#!/usr/bin/env python3
import argparse
import carla # pylint: disable=import-error
import math
import numpy as np
import time
import threading
from cereal import log
from multiprocessing import Process, Queue
from typing import Any

import cereal.messaging as messaging
from common.params import Params
from common.numpy_fast import clip
from common.realtime import Ratekeeper, DT_DMON
from lib.can import can_function
from selfdrive.car.honda.values import CruiseButtons
from selfdrive.test.helpers import set_params_enabled

parser = argparse.ArgumentParser(description='Bridge between CARLA and openpilot.')
parser.add_argument('--joystick', action='store_true')
parser.add_argument('--low_quality', action='store_true') #--low_quality
parser.add_argument('--town', type=str, default='Town04_Opt')
parser.add_argument('--spawn_point', dest='num_selected_spawn_point',
        type=int, default=16)

args = parser.parse_args()

W, H = 1164, 874
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100
STEER_RATIO = 15.

pm = messaging.PubMaster(['roadCameraState', 'sensorEvents', 'can', "gpsLocationExternal"])
sm = messaging.SubMaster(['carControl','controlsState','driverState','lateralPlan','longitudinalPlan','liveParameters', 'driverMonitoringState','liveLocationKalman'])

class VehicleState:
  def __init__(self):
    self.speed = 0
    self.angle = 0
    self.bearing_deg = 0.0
    self.vel = carla.Vector3D()
    self.cruise_button= 0
    self.is_engaged=False

def steer_rate_limit(old, new):
  # Rate limiting to 0.5 degrees per step
  limit = 0.5
  if new > old + limit:
    return old + limit
  elif new < old - limit:
    return old - limit
  else:
    return new

frame_id = 0
def cam_callback(image):
  global frame_id
  img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
  img = np.reshape(img, (H, W, 4))
  img = img[:, :, [0, 1, 2]].copy()

  dat = messaging.new_message('roadCameraState')
  dat.roadCameraState = {
    "frameId": image.frame,
    "image": img.tobytes(),
    "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  }
  pm.send('roadCameraState', dat)
  frame_id += 1

def imu_callback(imu, vehicle_state):
  vehicle_state.bearing_deg = math.degrees(imu.compass)
  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]
  pm.send('sensorEvents', dat)

def panda_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['pandaState'])
  while not exit_event.is_set():
    dat = messaging.new_message('pandaState')
    dat.valid = True
    dat.pandaState = {
      'ignitionLine': True,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hondaNidec'
    }
    pm.send('pandaState', dat)
    time.sleep(0.5)

def gps_callback(gps, vehicle_state):
  dat = messaging.new_message('gpsLocationExternal')

  # transform vel from carla to NED
  # north is -Y in CARLA
  velNED = [
    -vehicle_state.vel.y, # north/south component of NED is negative when moving south
    vehicle_state.vel.x, # positive when moving east, which is x in carla
    vehicle_state.vel.z,
  ]

  dat.gpsLocationExternal = {
    "timestamp": int(time.time() * 1000),
    "flags": 1, # valid fix
    "accuracy": 1.0,
    "verticalAccuracy": 1.0,
    "speedAccuracy": 0.1,
    "bearingAccuracyDeg": 0.1,
    "vNED": velNED,
    "bearingDeg": vehicle_state.bearing_deg,
    "latitude": gps.latitude,
    "longitude": gps.longitude,
    "altitude": gps.altitude,
    "speed": vehicle_state.speed,
    "source": log.GpsLocationData.SensorSource.ublox,
  }

  pm.send('gpsLocationExternal', dat)

def fake_driver_monitoring(exit_event: threading.Event):
  pm = messaging.PubMaster(['driverState','driverMonitoringState'])
  while not exit_event.is_set():
    # dmonitoringmodeld output
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    pm.send('driverMonitoringState', dat)

    time.sleep(DT_DMON)

def can_function_runner(vs: VehicleState, exit_event: threading.Event):
  i = 0
  while not exit_event.is_set():
    can_function(pm, vs.speed, vs.angle, i, vs.cruise_button, vs.is_engaged)
    time.sleep(0.01)
    i+=1


def bridge(q):

  # setup CARLA
  client = carla.Client("127.0.0.1", 2000)
  client.set_timeout(10.0)
  world = client.load_world(args.town)

  if args.low_quality:
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Particles)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)
  
  # change weather carla 
  #world.set_weather(carla.WeatherParameters.WetCloudySunset)

  blueprint_library = world.get_blueprint_library()

  world_map = world.get_map()

  vehicle_bp = blueprint_library.filter('vehicle.tesla.*')[1]
  spawn_points = world_map.get_spawn_points()
  assert len(spawn_points) > args.num_selected_spawn_point, \
    f'''No spawn point {args.num_selected_spawn_point}, try a value between 0 and
    {len(spawn_points)} for this town.'''
  spawn_point = spawn_points[args.num_selected_spawn_point]
  vehicle = world.spawn_actor(vehicle_bp, spawn_point)

  max_steer_angle = vehicle.get_physics_control().wheels[0].max_steer_angle

  # make tires less slippery
  # wheel_control = carla.WheelPhysicsControl(tire_friction=5)
  physics_control = vehicle.get_physics_control()
  physics_control.mass = 2326
  # physics_control.wheels = [wheel_control]*4
  physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
  physics_control.gear_switch_time = 0.0
  vehicle.apply_physics_control(physics_control)

  blueprint = blueprint_library.find('sensor.camera.rgb')
  blueprint.set_attribute('image_size_x', str(W))
  blueprint.set_attribute('image_size_y', str(H))
  blueprint.set_attribute('fov', '70')
  blueprint.set_attribute('sensor_tick', '0.05')
  transform = carla.Transform(carla.Location(x=0.8, z=1.13))
  camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
  camera.listen(cam_callback)

  vehicle_state = VehicleState()

  # reenable IMU
  imu_bp = blueprint_library.find('sensor.other.imu')
  imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
  imu.listen(lambda imu: imu_callback(imu, vehicle_state))

  gps_bp = blueprint_library.find('sensor.other.gnss')
  gps = world.spawn_actor(gps_bp, transform, attach_to=vehicle)
  gps.listen(lambda gps: gps_callback(gps, vehicle_state))


  #spawn leading vehicle 'this is carla code'
  lead_id = spawn_leading_vehicle(client, world,blueprint_library,vehicle)
  ##########################################################################################################


  # launch fake car threads
  threads = []
  exit_event = threading.Event()
  threads.append(threading.Thread(target=panda_state_function, args=(exit_event,)))
  threads.append(threading.Thread(target=fake_driver_monitoring, args=(exit_event,)))
  threads.append(threading.Thread(target=can_function_runner, args=(vehicle_state, exit_event,)))
  for t in threads:
    t.start()

  # can loop
  rk = Ratekeeper(100, print_delay_threshold=0.05)

  # init
  throttle_ease_out_counter = REPEAT_COUNTER
  brake_ease_out_counter = REPEAT_COUNTER
  steer_ease_out_counter = REPEAT_COUNTER


  vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)

  is_openpilot_engaged = False
  throttle_out = steer_out = brake_out = 0
  throttle_op = steer_op = brake_op = 0
  throttle_manual = steer_manual = brake_manual = 0

  old_steer = old_brake = old_throttle = 0
  throttle_manual_multiplier = 0.7 #keyboard signal is always 1
  brake_manual_multiplier = 0.7 #keyboard signal is always 1
  steer_manual_multiplier = 45 * STEER_RATIO  #keyboard signal is always 1


  old_steering =0 

  while 1:
    # 1. Read the throttle, steer and brake from op or manual controls
    # 2. Set instructions in Carla
    # 3. Send current carstate to op via can

    cruise_button = 0
    throttle_out = steer_out = brake_out = 0.0
    throttle_op = steer_op = brake_op = 0
    throttle_manual = steer_manual = brake_manual = 0.0

    # --------------Step 1-------------------------------
    if not q.empty():
      message = q.get()
      m = message.split('_')
      if m[0] == "steer":
        steer_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "throttle":
        throttle_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "brake":
        brake_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "reverse":
        #in_reverse = not in_reverse
        cruise_button = CruiseButtons.CANCEL
        is_openpilot_engaged = False
      elif m[0] == "cruise":
        if m[1] == "down":
          cruise_button = CruiseButtons.DECEL_SET
          is_openpilot_engaged = True
        elif m[1] == "up":
          cruise_button = CruiseButtons.RES_ACCEL
          is_openpilot_engaged = True
        elif m[1] == "cancel":
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False
      elif m[0] == "quit":
        break

      throttle_out = throttle_manual * throttle_manual_multiplier
      steer_out = steer_manual * steer_manual_multiplier
      brake_out = brake_manual * brake_manual_multiplier

      #steer_out = steer_out
      # steer_out = steer_rate_limit(old_steer, steer_out)
      old_steer = steer_out
      old_throttle = throttle_out
      old_brake = brake_out

      # print('message',old_throttle, old_steer, old_brake)

    if is_openpilot_engaged:
      sm.update(0)
      # TODO gas and brake is deprecated
      throttle_op = clip(sm['carControl'].actuators.accel/4.0, 0.0, 1.0)
      brake_op = clip(-sm['carControl'].actuators.accel/4.0, 0.0, 1.0)
      steer_op = sm['carControl'].actuators.steeringAngleDeg

      throttle_out = throttle_op
      steer_out = steer_op
      brake_out = brake_op

      steer_out = steer_rate_limit(old_steer, steer_out)
      old_steer = steer_out

    else:
      if throttle_out==0 and old_throttle>0:
        if throttle_ease_out_counter>0:
          throttle_out = old_throttle
          throttle_ease_out_counter += -1
        else:
          throttle_ease_out_counter = REPEAT_COUNTER
          old_throttle = 0

      if brake_out==0 and old_brake>0:
        if brake_ease_out_counter>0:
          brake_out = old_brake
          brake_ease_out_counter += -1
        else:
          brake_ease_out_counter = REPEAT_COUNTER
          old_brake = 0

      if steer_out==0 and old_steer!=0:
        if steer_ease_out_counter>0:
          steer_out = old_steer
          steer_ease_out_counter += -1
        else:
          steer_ease_out_counter = REPEAT_COUNTER
          old_steer = 0

    # --------------Step 2-------------------------------

    steer_carla = steer_out / (max_steer_angle * STEER_RATIO * -1)

    steer_carla = np.clip(steer_carla, -1,1)
    steer_out = steer_carla * (max_steer_angle * STEER_RATIO * -1)
    old_steer = steer_carla * (max_steer_angle * STEER_RATIO * -1)

    vc.throttle = throttle_out/0.6
    vc.steer = steer_carla
    vc.brake = brake_out
    vehicle.apply_control(vc)

    # --------------Step 3-------------------------------
    vel = vehicle.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) # in m/s
    vehicle_state.speed = speed
    vehicle_state.vel = vel
    vehicle_state.angle = steer_out
    vehicle_state.cruise_button = cruise_button
    vehicle_state.is_engaged = is_openpilot_engaged

    # compute distance to leading vehicle 
    dis, lead_location = distanceToLeadingVehicle(world, lead_id, vehicle)
    # this is a place of policing functions
    row_perstep = policing_function(vehicle_state.speed * 2.23694, vehicle_state.vel, steer_op ,old_steering, is_openpilot_engaged,vehicle.get_transform().location, dis, lead_location)
    old_steering = vehicle_state.angle

    if rk.frame%PRINT_DECIMATION == 0:
      print("frame: ", "engaged:", is_openpilot_engaged, "; throttle: ", round(vc.throttle, 3), "; steer(c/deg): ", round(vc.steer, 3), round(steer_out, 3), "; brake: ", round(vc.brake, 3))
      ###############################################################################################################
      csv_data.append(row_perstep)
      # save data to CSV file
      safeData(header,csv_data)
      #new_vehicle

    rk.keep_time()

  # Clean up resources in the opposite order they were created.
  exit_event.set()
  for t in reversed(threads):
    t.join()
  gps.destroy()
  imu.destroy()
  camera.destroy()
  vehicle.destroy()

header = ['time',
'is_openpilot_engaged',
'target position',
'LeadingVPosition',
'currentspeed',
'currentVelocity',
'current_Steer',
'old_Steer',
'steer_rate_limit',
'leftLaneProb',
'rightLaneProb',
'desiredPathProb',
'HasLead',
'DistanceToLead',
'specifyCruise', 
'awarenessStatus'
]
csv_data =[]

def policing_function(currentspeed, currentVelocity, current_Steer,old_Steer,is_openpilot_engaged, location, dis, lead_location):
  row_perstep = []

  ####  lateralPlan features #####################################################
  leftProb, rightProb , desiredPathPro , desire = lateralPlanOutputs()

  ####### longtiudanla features ######################################
  cruise_control_based, hasLead = longtiudinalPlan()

  ####  liveParameters features #####################################################
  valid, yawRate, steerRatio, sensorValid, angleOffsetAverageDeg, angleOffsetDeg = liveParametersOutputs()
  #print ('valid:', valid , 'yawRate:', yawRate, 'steerRatio:', steerRatio , 'angleOffsetAverageDeg:', angleOffsetAverageDeg)

  #### human fallback features ######################################################
  awarenessStatus , awarenessActive , awarenessPassive = humanDriverStateOutputs (0)

  ### cotrol parameters ###########################################################
  row_perstep.append(getTime())
  row_perstep.append(is_openpilot_engaged)
  row_perstep.append(location)
  row_perstep.append(lead_location)
  row_perstep.append(currentspeed)
  row_perstep.append(currentVelocity)
  row_perstep.append(round(current_Steer, 3))
  row_perstep.append(round(old_Steer, 3))
  row_perstep.append(0.5)
  row_perstep.append(round(leftProb, 3))
  row_perstep.append(round(rightProb, 3))
  row_perstep.append(round(desiredPathPro, 3))
  row_perstep.append(hasLead)
  row_perstep.append(dis)
  row_perstep.append(cruise_control_based)
  row_perstep.append(awarenessStatus)

  old_Steer = current_Steer



  return row_perstep

def lateralPlanOutputs ():
  leftProb = 0.0
  rightProb = 0.0

  desiredPathPoints = []
  #[0.073207125, 0.073198624, 0.0738575, 0.075219847, 0.076821364, 0.078002237,
   #0.080016986, 0.082570061, 0.085408896, 0.088324428, 0.091540441, 0.093923733,
    #0.094881013, 0.093809247, 0.085320882, 0.06568329, 0.031407647]

  desiredPathPro = 0.0
  #desire = []
  curvatures = []
  #[0.0010510444, 0.0010572524, 0.0010730366, 0.0010903559, 0.0010971641, 0.0010822284,
   #0.0010328537, 0.00093945896, 0.00079797831, 0.00061710551, 0.00040210746, 0.00016967548,
    #-5.8945054e-05, -0.00026038621, -0.000409916, -0.000497971, -0.00052599644]
  curvatureRates = []
    #[0.00032184258, 0.00045741361, 0.00070415885, 0.00097288738, 0.0012790792, 0.0015310304,
   #0.0016727661, 0.0016703397, 0.0015062874, 0.0012670484, 0.00096328289, 0.00065119448, 0.0003742766,
    #0.00017187183, 5.72019e-05, 7.9354322e-06, 0]

  
  #sm.update(3)
  leftProb = sm['lateralPlan'].lProb
  rightProb = sm['lateralPlan'].rProb

  desiredPathPoints = sm['lateralPlan'].dPathPoints
  desiredPathPro = sm['lateralPlan'].dProb
  desire = sm['lateralPlan'].desire

  curvatures = sm['lateralPlan'].curvatures
  curvatureRates = sm['lateralPlan'].curvatureRates

  return leftProb, rightProb , desiredPathPro, desire


def longtiudinalPlan():
  sm.update(4)

  hasLead = sm['longitudinalPlan'].hasLead
  speeds = sm['longitudinalPlan'].speeds
  accels = sm['longitudinalPlan'].accels
  jerks = sm['longitudinalPlan'].jerks

  vCruiseDEPRECATED = sm ['longitudinalPlan'].vCruiseDEPRECATED
  cruise_control_based = sm ['longitudinalPlan'].longitudinalPlanSource

  return cruise_control_based, hasLead

def liveParametersOutputs():

  sm.update(5)
  valid = sm['liveParameters'].valid
  yawRate = sm['liveParameters'].yawRate
  steerRatio = sm['liveParameters'].steerRatio
  sensorValid = sm['liveParameters'].sensorValid
  angleOffsetAverageDeg = sm['liveParameters'].angleOffsetAverageDeg
  angleOffsetDeg = sm['liveParameters'].angleOffsetDeg
  return valid, yawRate, steerRatio, sensorValid, angleOffsetAverageDeg, angleOffsetDeg


def humanDriverStateOutputs (humanFallbackMode):
  if humanFallbackMode ==1:
    # senstiveFeature
    sm.update(2)
    faceProb = 0.0
    leftEyeProb = 0.0
    rightEyeProb = 0.0
    leftBlinkProb = 0.0
    rightBlinkProb = 0.0

    distraxtedProb = 0.0
    distractedEyes = 0.0
    eyesOnRoad = 0.0
    phoneUse = 0.0

    sm.update(2)
    faceProb = sm['driverState'].faceProb
    leftEyeProb = sm['driverState'].leftEyeProb 
    rightEyeProb = sm['driverState'].rightEyeProb 
    leftBlinkProb = sm['driverState'].leftBlinkProb 
    rightBlinkProb = sm['driverState'].rightBlinkProb

    distraxtedProb = sm['driverState'].distraxtedProb 
    distractedEyes = sm['driverState'].distractedEyes 
    eyesOnRoad = sm['driverState'].eyesOnRoad 
    phoneUse = sm['driverState'].phoneUse

  sm.update(6)
  awarenessStatus = sm['driverMonitoringState'].awarenessStatus
  awarenessActive = sm['driverMonitoringState'].awarenessActive 
  awarenessPassive = sm['driverMonitoringState'].awarenessPassive 
  return awarenessStatus , awarenessActive , awarenessPassive


def carStateParameters ():
  sm.update(1)
  awarenessStatus = sm['controlsState'].awarenessStatus

  openPilotstate = sm['controlsState'].OpenpilotState
  vTargetLead = sm['controlsState'].vTargetLead
  vCruise = sm['controlsState'].vCruise
  aTarget = sm['controlsState'].aTarget

  #UI tests
  engageable = sm['controlsState'].engageable # user can run OP
  alertStatus = sm['controlsState'].AlertStatus # user can run OP
  alertText1 =  sm['controlsState'].alertText1
  alertText2 =  sm['controlsState'].alertText2
  alertSound = sm['controlsState'].alertSound

  return True


from datetime import datetime
def getTime():
  now = datetime.now()
  current_time = now.strftime("%H+1:%M:%S")
  return current_time

import csv
def safeData(header, data):
  with open('c1.csv', 'w', encoding='UTF8', newline='') as f:
    writer = csv.writer(f)
    # write the header
    writer.writerow(header)
    # write multiple rows
    writer.writerows(data)
  f.close()
  print('DataSaved')

def spawn_leading_vehicle(client, world, blueprint_library, vehicle):
    """ This function spawns the leading vehicle and puts it into
    an autopilot mode.
    """
    # Get the spawn point of the ego vehicle.
    vehicle_location = vehicle.get_transform().location
    vehicle_waypoint = world.get_map().get_waypoint(vehicle_location)

    # spawn leading vehicle infront of ego vehicle
    new_vehicle_waypoint = vehicle_waypoint.next(20)[0]
    new_vehicle_location = new_vehicle_waypoint.transform.location + carla.Location(0, 0 , 2)
    new_vehicle_rotation = new_vehicle_waypoint.transform.rotation
    new_vehicle = blueprint_library.filter('vehicle.tesla.*')[1]
    #world.try_spawn_actor(new_vehicle, carla.Transform(new_vehicle_location, new_vehicle_rotation))

    #spawn a leading vehicle and make it in autopilotmode --> this added by carla code by carla
    batch = [
        carla.command.SpawnActor(new_vehicle, carla.Transform(new_vehicle_location, new_vehicle_rotation)).then(
            carla.command.SetAutopilot(carla.command.FutureActor, True))
    ]
    lead_vehicle_id = client.apply_batch_sync(batch)[0].actor_id
    time.sleep(0.5)

    return lead_vehicle_id


import numpy as np
def distanceToLeadingVehicle(world , lead_id , ego_vehicle):
  actors = world.get_actors().filter('vehicle.*')
  
  lead_vehicle = None
  # get leading vehicle 
  for actor in world.get_actors().filter('vehicle.*'):
    if actor.id == lead_id:
      lead_vehicle = actor
      break

  lead_location = lead_vehicle.get_location()
  ego_location = ego_vehicle.get_location()


  print('leadLocation::',lead_location )
  print('ego_vehicle_location::',ego_location )
  
  p1 = np.array([lead_location.x, lead_location.y, lead_location.z])
  p2 = np.array([ego_location.x, ego_location.y, ego_location.z])
  
  # square diff between Xs, Ys and Zs  
  #squared_dist = np.sum((p1-p2)**2, axis=0)
  # then squre to remove - 
  #dist = np.sqrt(squared_dist)

  dist = carla.Location.distance(lead_location,ego_location )

  #print('distttttttt:::',dist) # distance in metter according to carla
  
  return dist, lead_location




import cv2
def readFaceImagefromPCCamera():
  cv2.namedWindow("preview")
  vc_cam = cv2.VideoCapture(0)
  #read face camera 
  if vc_cam.isOpened(): # try to get the first frame
    rval, frame = vc.read()
  else:
    rval = False
  if rval:
    cv2.imshow("preview", frame)
    rval, frame = vc_cam.read()
    print('face detection')
  else:
    print('No face detection')



def bridge_keep_alive(q: Any):
  while 1:
    try:
      bridge(q)
      break
    except RuntimeError:
      print("Restarting bridge...")

if __name__ == "__main__":
  # make sure params are in a good state
  set_params_enabled()

  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  Params().put("CalibrationParams", msg.to_bytes())

  q: Any = Queue()
  p = Process(target=bridge_keep_alive, args=(q,), daemon=True)
  p.start()

  if args.joystick:
    # start input poll for joystick
    from lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(q)
    p.join()
  else:
    # start input poll for keyboard
    from lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(q)
