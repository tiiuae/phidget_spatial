#!/bin/bash

mkdir -p /etc/udev/rules.d

cat << EOF > /etc/udev/rules.d/99-phidgets.rules

# Very old Phidgets
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="0925", ATTRS{idProduct}=="8101", MODE="666"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="0925", ATTRS{idProduct}=="8104", MODE="666"
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="0925", ATTRS{idProduct}=="8201", MODE="666"

# All current and future Phidgets - Vendor = 0x06c2, Product = 0x0030 - 0x00af
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="06c2", ATTRS{idProduct}=="00[3-a][0-f]", MODE="666"

EOF

chmod 644 /etc/udev/rules.d/99-phidgets.rules

echo "Please input correct drone ID and phidget serial numbers into /opt/ros/foxy/lib/phidget_spatial/phidget_spatial.yaml"

cat << EOF > /etc/systemd/system/phidget_spatial.service

[Unit]
Description=Phidget Spatial service
StopWhenUnneeded=true

[Service]
Type=simple
User=sad
Group=sad
Restart=always
RestartSec=5
ExecStart=/bin/sh -c ". /opt/ros/foxy/setup_fog.sh; ros2 launch phidget_spatial phidget_spatial_launch.py"

[Install]
WantedBy=multi-user.target

EOF

exit 0
