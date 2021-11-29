# phidget_spatial
ROS2 node for handling multiple Phidget Spatial sensors

* Publishes the sensor data from all configured Phidget Spatial sensors. 
* Publish interval is parameterised, and can be any integer between 4 and 1000 (ms), inclusive. 
* Can support an arbitrary amount of devices.
* Publishes to two topics: one for acceleration and gyroscope data, and the other for magnetometer data.

# Installation
Before installing the debian package, run the following commands to install dependencies:
```
sudo apt-get install -y wget && \
    wget -qO- http://www.phidgets.com/gpgkey/pubring.gpg | apt-key add - && \
    echo 'deb http://www.phidgets.com/debian bullseye main' > /etc/apt/sources.list.d/phidgets.list

sudo apt-get update
sudo apt-get install libphidget22

pip install Phidget22
```

Use `apt` to install the package, and after installation, edit the config file to match the proper drone device ID and Phidget
serial numbers. The serial numbers are given as a list, order should correspond to the numbering of the motors. Set the publish 
interval to the required number.

Installation also creates a `systemd service` file. The systemd service file is created here 
instead of `fogsw_systemd` since the package is likely to be rarely installed.  

Finally, run `sudo systemctl enable phidget_spatial.service` before rebooting.