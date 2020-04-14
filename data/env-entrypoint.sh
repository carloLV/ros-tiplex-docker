#!/bin/bash

cd ~ && \
    mkdir -p ./catkin_ws/src && \
    cd catkin_ws/src && \
    ln -s /app/tbot_testing_env && \
    ln -s /app/robot_knowledge_base && \
    cp /app/start_node* ../ && \
    chmod +x ../../start_node* && \
    cd robot_knowledge_base/src/ && \
    chmod +x back_robot_side.py back_planner_side.py && \
    cd ~/catkin_ws && \
    catkin_make
