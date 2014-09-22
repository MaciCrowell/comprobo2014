#!/bin/bash

source /opt/ros/hydro/setup.bash
rostopic pub --once /pi_cmd std_msgs/String normalwireless

echo "Starting up normal wireless"
echo "Make sure to use sudo when executing"

start network-manager
