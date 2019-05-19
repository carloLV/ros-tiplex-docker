#!/bin/bash

echo 'running ROSCORE'
xterm -hold -e 'cd ./catkin_ws && source devel/setup.bash && roscore' &

# Wait for master to be up
sleep 3

echo 'Running MONGODB node'
xterm -hold -e 'cd ./catkin_ws && source devel/setup.bash && rosparam set mongodb_port 62345 && rosparam set mongodb_host dev && rosrun mongodb_store mongodb_server.py' &

echo 'Running TBOT_ENV node'
xterm -hold -e 'cd ./catkin_ws && source devel/setup.bash &&' &
