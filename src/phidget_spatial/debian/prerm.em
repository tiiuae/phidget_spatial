#!/bin/bash

if [ -e /etc/systemd/system/phidget_spatial.service ]; then
    rm /etc/systemd/system/phidget_spatial.service
fi


exit 0
