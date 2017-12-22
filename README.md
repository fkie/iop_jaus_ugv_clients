See [iop_core](https://github.com/fkie/iop_core/blob/master/README.md) for use instructions.


# Interfaces

The repository contains clients designed to control services on IOP complient robot. All client services are based on ```SlaveHandlerInterface``` and use funtionality of [Slave](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#iop_ocu_slavelib_fkie).  

List of service plugins in this repository:

[iop_client_stabilizer_driver_fkie: StabilizerDriverClient](#iop_client_stabilizer_driver_fkie-stabilizersriverclient)  


## _iop_client_stabilizer_driver_fkie:_ StabilizerDriverClient

A simple interface to control a set of flipper of a robot by velocity effort or position. The flippers can be controlled by _sensor_msgs::JointState_ or _std_msgs::Float64MultiArray_ messages. Both messages are published, so be sure to subscribe only one of them.

#### Parameter:

_hz (int_ Default: 1.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/doc/iop_core_packages.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.

_joint_names (list_ Default: [])

> Specifies a list with joint names. This is important to identify the positon in JointState messages. If no names are specified they will be generated from reported capabilities from the robot.

#### Publisher:

_joint_states (sensor_msgs::JointState)_

> Reports the position and/or velocity.

#### Subscriber:

_cmd_joint_states (sensor_msgs::JointState)_

> Control flipper by position or velocity. For joint names see the parameter _joint_names_.

_flipper_velocity_controller/command (std_msgs::Float64MultiArray)_

> Control flipper by velocity. The count of values should be lesser than reported from robot.

