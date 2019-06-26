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
                chown developer: /opt/ros/mongodb_store && \
                chmod u+w /opt/ros/mongodb_store && \
                rosrun mongodb_store mongodb_server.py' &
                
echo 'MONGODB node is up!!\n'

# Wait for MONGODB to be up
sleep 3

echo 'Running TEST_BOT node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch tbot_testing_env tbot_testing.launch' &
                
echo 'TEST_BOT node is up!!\n'

# Wait for TEST ENV to be up
sleep 3

echo 'Running ROBOT_KNOWLEDGE_BASE node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch robot_knowledge_base knowledge_base_robot.launch' &
                
echo 'ROBOT_KNOWLEDGE_BASE node is up!!\n'

sleep 3

echo 'Running PLANNER_KNOWLEDGE_BASE node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch robot_knowledge_base knowledge_base_planner.launch' &
                
echo 'PLANNER_KNOWLEDGE_BASE node is up!!\n'
