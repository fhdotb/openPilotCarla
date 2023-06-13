<p><b>Data</b></p>
<p>This folder contains extracted data and recorded videos from OpenPilot Software [<a href="https://github.com/commaai/openpilot">(1)</a>] in the Carla driving simulator [<a href="https://github.com/carla-simulator/carla">(2)</a>]. Carla's Town 04 was used as the driving environment for exploring and testing OpenPilot's behaviors. Town 04, which is a highway, provides a suitable environment for the Automated Lane Centering (ALC) functionalities of OpenPilot. Moreover, the collected data is categorized based on different weather conditions ('clear,' 'rainy'), with the self-driving vehicle (SDV) speed adjusted in various settings such as 25, 30, and 40 miles per hour.</p>

<p><b>The structure of datasets (CSV files) is as follows:</b></p>

<pre>
'time': # The timestamp of when a driving decision is made, corresponding to the recorded data. 
'is_openpilot_engaged': # checks whether OpenPilot is engaged
'target position': # position the ego-vehicle attempts to reach
'currentspeed': # current speed of the SDV in its environment
'currentVelocity': # current velocity of the SDV
'current_vehicle_angle': # current steering angle of the physical wheels (between 70 to -70)
'old_vehicle_angle': # detects previous steering angle of the SDV
'max_steer_vehicle_angle': # maximum steering angle that the SDV's wheels can reach
'steer_rate_limit': # acceptable amount of change between the old and current steering angle
'current_steer': # current yaw angle (between 1 to -1)
'old_steer': # previous yaw angle
'leftLaneProb': # detection probability of the left lane line (between 0-100)
'rightLaneProb': # detection probability of the right lane line (between 0-100)
'desiredPathProb': # detection probability of the desired path (between 0-100)
'HasLead': # boolean value indicating the detection of a leading vehicle (False: no front vehicle, True: yes)
'specifyCruise': # speed control by setSpeed or control by the leading vehicle
'awarenessStatus': # indicates if the driver is providing human-monitored features (hand-on steering wheel)
'alertStatus': # alert status (normal, userPrompt, critical) indicating if the system wants to alert the driver
'alertText1': # alert text 1
'alertText2': # alert text 2
'alertSound': # sound notification
'detectionProbability': # detection probability of the desired path more than 60, less than 30, or between them
'safety_procedure': # the mitigation strategy must be here
</pre>

<p><b>Python Scripts</b></p>
<p>This folder contains Python scripts designed to extract OpenPilot behaviors and record videos within the Carla simulator. These scripts are based on the plug-in (bridge) provided by OpenPilot, which can be found at the following <a href="https://github.com/commaai/openpilot/blob/master/tools/sim/bridge.py">link</a>.</p>

<p><b>Instructions to run the Jupyter notebook</b></p>
<ol>
  <li>Download Anaconda 3 from <a href="https://www.anaconda.com/products/individual">(3)</a>.</li>
  <li>Create a new conda environment and install 'environment_root.yml' provided with this project. This can be done with the following command, as explained <a href="https://stackoverflow.com/questions/58272405/how-to-install-packages-from-yaml-file-in-conda">here</a>: <code>conda env update -n my_env --file ENV.yaml</code></li>
</ol>
