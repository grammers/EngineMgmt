# engine_mgmt
Engine controller for [PermoCar](https://github.com/Trobolit/PermoCar) project.
In these node is the control lope placed and there for are feedback and stop signals fed to it along with driving commands.

## Subscribe
* topic joy, sensor_msgs/joy (message form controller input)
* topic stop, std_msgs/Bool (collision warning)
* topic wheel_velocity, std_msgs/Float32MultiArray (feedback form wheel ass L/R speed)
* topic vw_estimate, std_msgs/Float32MultiArray (feedback from wheel in v and w)

## Publich
* topic motor_power, geometry_msgs/Twist (desired power to engine)


## Param
Params that can be set and their default values.
* power_buffer_size int 200
* encoder_buffer_size int 5
* loop_freq int 20
* time_out int 500 (unit ms)
* k11 float 2.5
* kr11 float 2.5
* k22 float 6
* kr22 float 10
