#!/bin/bash

source /opt/ros/hydro/setup.bash
rostopic pub --once /pi_cmd std_msgs/String shutdown
