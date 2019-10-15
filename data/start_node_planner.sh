#!/bin/bash

echo 'running ROSCORE'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roscore' &

echo 'ROSCORE node is up!!\n'

# Wait for master to be up
sleep 3

echo 'Running MONGODB node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                rosparam set mongodb_port 62345 && \
                rosparam set mongodb_host localhost && \
                rosrun mongodb_store mongodb_server.py' &

# Add this 2 lines to the command above in case this is the first run of MongoDB in the docker
## chown developer: /opt/ros/mongodb_store && \
## chmod u+w /opt/ros/mongodb_store && \

echo 'MONGODB node is up!!\n'

# Wait for MONGODB to be up

sleep 3

echo 'Running PLANNER_KNOWLEDGE_BASE node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch robot_knowledge_base knowledge_base_planner.launch' &

echo 'PLANNER_KNOWLEDGE_BASE node is up!!\n'

# Wait for planner node to be up

sleep 3

echo 'Running python server on port 9000'
xterm -hold -e 'cd /app/planning_interface && \
                python -m SimpleHTTPServer 9000' &

echo 'Python server is up!!\n'
