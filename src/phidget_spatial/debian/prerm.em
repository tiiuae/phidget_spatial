#!/bin/bash

if [ -e /etc/udev/rules.d/99-phidgets.rules ]; then
    rm /etc/udev/rules.d/99-phidgets.rules
fi

exit 0
