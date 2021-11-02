# phidget_spatial
ROS2 node for handling multiple Phidget Spatial sensors

* Publishes the sensor data from all configured Phidget Spatial sensors. 
* Publish interval is parameterised, and can be any integer between 4 and 1000 (ms), inclusive. 
* Can support an arbitrary amount of devices.
* Publishes to two topics: one for acceleration and gyroscope data, and the other for magnetometer data.

# Installation
When installing from the debian package, edit the config file to match the proper drone device ID and Phidget
serial numbers. The serial numbers are given as a list, order should correspond to the numbering of the motors.

Installation also creates a `rules.d` file and `systemd service` files. The systemd service files are created here 
instead of `fogsw_systemd` since the package is likely to be installed rarely.  