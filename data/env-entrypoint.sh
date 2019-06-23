#!/bin/bash

cd ~ && \
    mkdir -p ./catkin_ws/src && \
    cd catkin_ws && \
    cp -r /app/tbot_testing_env ./src && \
    cp -r /app/robot_knowledge_base ./src && \
    cp /app/start_nodes.sh ../ && \
    chmod +x ../start_nodes.sh && \
    cd src/robot_knowledge_base/src/ && \
    chmod +x back_robot_side.py back_planner_side.py && \
    cd ~/catkin_ws && \
    catkin_make
