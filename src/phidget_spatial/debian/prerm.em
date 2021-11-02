#!/bin/bash

if [ -e /etc/udev/rules.d/99-phidgets.rules ]; then
    rm /etc/udev/rules.d/99-phidgets.rules
fi

if [ -e /etc/systemd/system/phidget_spatial.service ]; then
    rm /etc/systemd/system/phidget_spatial.service
fi


exit 0
