#!/bin/bash
if [ -e "/tmp/checkout/setup.bash" ]; then source /tmp/checkout/setup.bash; fi
roslaunch squirrel_website build.launch
