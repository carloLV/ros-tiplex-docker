#!/bin/bash

echo 'running ROSCORE'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roscore' &

echo 'ROSCORE node is up!!\n'

# Wait for master to be up
sleep 3

echo 'Running TEST_BOT node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch tbot_testing_env tbot_testing.launch' &

echo 'TEST_BOT node is up!!\n'

# Wait for TEST ENV to be up
sleep 20

echo 'Running ROBOT_KNOWLEDGE_BASE node'
xterm -hold -e 'cd ./catkin_ws && \
                source devel/setup.bash && \
                roslaunch robot_knowledge_base knowledge_base_robot.launch' &

echo 'ROBOT_KNOWLEDGE_BASE node is up!!\n'

# Wait for robot node to be up

sleep 3

echo 'Running python server on port 9000'
xterm -hold -e 'cd /app/planning_interface && \
                python -m SimpleHTTPServer 9000' &

echo 'Python server is up!!\n'
