# force_emergency_stop
When excessive force is applied to the end-effector or a service is received, the currently executing manipulator plan will be stopped.

## install 
```
git clone https://github.com/RDC-harvesting-robot/leptrino_force_sensor.git
```

## Execute
```
ros2 launch force_emergency_stop force_emargency_stop.launch.py port:=/dev/ttyACM2
```
service call
```
ros2 service call /cancel_trajectory std_srvs/srv/Trigger
```
