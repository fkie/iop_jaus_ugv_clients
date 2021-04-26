This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _fkie_iop_client_illumination:_ IlluminationClient

Control the lights on the robot. For each supported light a publisher for state and a subscriber for commands are created. Possible supported lights are:

    - head_lights
    - left_turn_signal
    - right_turn_signal
    - running_lights
    - brake_lights
    - backup_lights
    - visible_light_source
    - ir_light_source
    - variable_light_1
    - variable_light_2
    - variable_light_3
    - variable_light_4
    - high_beams
    - parking_lights
    - fog_lights
    - hazard_lights


#### Parameter:

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/fkie_iop_ocu_slavelib/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_illuminator/`key` (std_msgs::msg::Bool)_

> State for supported light.

_illuminator_states (diagnostic_msgs/DiagnosticStatus)_

> A list with supported lights and their states.


#### Subscriber:

_illuminator/cmd_`key` (std_msgs::msg::Bool)_

> Command for supported light.

