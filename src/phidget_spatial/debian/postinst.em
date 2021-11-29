#!/bin/bash

echo "Please input correct drone ID and phidget serial numbers into /opt/ros/foxy/config/phidget_spatial/phidget_spatial.yaml"

cat << EOF2 > /etc/systemd/system/phidget_spatial.service

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

EOF2

exit 0
