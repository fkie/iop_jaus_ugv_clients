This package is part of [ROS/IOP Bridge](https://github.com/fkie/iop_core/blob/master/README.md).


## _iop_client_power_plant_fkie:_ PowerPlantManagerClient

Currently publish reports the status of one battery.


#### Parameter:

_hz (int_ Default: 0.0)

> Sets how often the reports are requested. If [use_queries](https://github.com/fkie/iop_core/blob/master/iop_ocu_slavelib_fkie/README.md#parameter) is ```True``` hz must be greather then 0. In this case each time a ```Query``` message is sent to get a report. If ```use_queries``` is ```False``` an event is created to get Reports. In this case 0 disables the rate and an event of type ```on_change``` will be created.


#### Publisher:

_powerplant\_`ID`/voltage (std_msgs::Float32)_

> Current voltage.

_powerplant\_`ID`/capacity_percent (std_msgs::Int8)_

> Percent of maximum.


#### Subscriber:

> None

