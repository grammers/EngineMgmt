# engine_mgmt
Engien controler for [PermoCar](https://github.com/Trobolit/PermoCar) prodjek.
Thise node is created to calibrate the speed to the wheel.


## Subscribe
* topic joy, sensor_msgs/joy (message form controller input)
* topic stop, std_msgs/Bool (collition vorning)
* topic wheel_velocity, std_msgs/Float32MultiArray (fead back form wheel)

## Publich
* topic motor_power, geometry_msgs/Twist (desierd power to engine)
