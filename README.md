<p>
This repository presents a comprehensive exploration of OpenPilot's Automated Lane Centering (ALC) system within the Carla simulator. We've conducted a detailed study of the ALC system's responses to various conditions, including dynamic weather and differing target speeds. Our findings are demonstrated through recorded videos and CSV files, illustrating the ALC system's operational methodology. For instance, the ALC system suggests desired driving paths by identifying left and right lane lines. Our observation process includes tracking these detection results, expressed as probabilities. Each detection event is linked to a corresponding frame in the video, offering a clear visual representation of the ALC system's decision-making process.
</p>

<p><b>Data</b></p>
<p>
This folder contains extracted data and recorded videos from OpenPilot Software [<a href="https://github.com/commaai/openpilot">openPilot</a>] in the Carla driving simulator [<a href="https://github.com/carla-simulator/carla">Carla</a>]. Carla's Town 04, which features a highway, was used as the driving environment for exploring and testing OpenPilot's behaviors. This environment provides an ideal setting for the Automated Lane Centering (ALC) functionalities of OpenPilot. Furthermore, the collected data is categorized based on various weather conditions ('clear,' 'rainy') and adjusted vehicle speeds such as 25, 30, and 40 miles per hour.
</p>

<p><b>The structure of datasets (CSV files) is as follows:</b></p>

<pre>
'time': The timestamp of a driving decision, corresponding to the recorded data. 
'is_openpilot_engaged': Indicator of when OpenPilot is ready to engage/work.
'target position': Position that the SDV aims to reach.
'currentspeed': Current speed of the SDV.
'currentVelocity': Current velocity of the SDV.
'current_vehicle_angle': Current steering angle of the physical wheels (range from -70 to 70).
'old_vehicle_angle': Previous steering angle of the SDV.
'max_steer_vehicle_angle': Maximum steering angle that the SDV's wheels can reach.
'steer_rate_limit': Acceptable amount of change between the old and current steering angle.
'current_steer': Current yaw angle (range from -1 to 1).
'old_steer': Previous yaw angle.
'leftLaneProb': Detection probability of the left lane line (range from 0-100).
'rightLaneProb': Detection probability of the right lane line (range from 0-100).
'desiredPathProb': Detection probability of the desired path (range from 0-100).
'HasLead': Boolean value indicating the detection of a leading vehicle (False: there is no leading vehicle, True: there is leading vehicle ).
'specifyCruise': Speed control determined by setSpeed or control by the leading vehicle.
'awarenessStatus': Indicates if the driver is providing human-monitored features (hand-on steering wheel, sensitive features, i.e., eyes, head and etc).
'alertStatus': Alert status (normal, userPrompt, critical) indicating if the system wants to alert the driver.
'alertText1': First alert text.
'alertText2': Second alert text.
'alertSound': Sound notification.
</pre>


<p><b>Python Scripts</b></p>
<p>
This folder contains Python scripts designed to extract OpenPilot behaviors and record videos within the Carla simulator. These scripts are based on the plug-in (bridge) provided by OpenPilot, which can be found at the <a href="https://github.com/commaai/openpilot/blob/master/tools/sim/bridge.py"> Openpilot bridge</a>.
</p>

<p><b>Demo</b></p>
<p>
The Jupyter notebook available at this <a href="https://git.soton.ac.uk/faa2n19/openpilotcarla/-/blob/master/Demo%20of%20analysing%20OpenPilot%20behaviours%20in%20Carla%20town%204.ipynb">link</a> provides a demonstration of the behavior of the OpenPilot software within the Carla Simulator environment.
</p>


<p><b>Instructions to Run the Jupyter Notebook</b></p>
<ol>
  <li>Download Anaconda 3 from <a href="https://www.anaconda.com/products/individual">anaconda</a>.</li>
  <li>Create a new conda environment and install the 'environment_root.yml' file provided with this project. This can be done using the following command, as explained in the <a href="https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html">conda documentation</a>: 
  <ol>
    <li><code>conda env update -n my_env --file environment_root.yml</code></li>
  </ol>
  </li>
</ol>
