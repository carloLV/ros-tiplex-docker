FROM ros:indigo-robot

# Install needed software
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y  \
    build-essential \
    nano \
    ros-indigo-rosbridge-server \
    ros-indigo-husky-simulator \
    ros-indigo-husky-navigation \
    ros-indigo-husky-desktop \
    python-pymongo \
    mongodb \
    ros-indigo-mongodb-store
    
# Copy our code from the current folder to /app inside the container
RUN mkdir /app
ADD ./data /app
    
# Set up forwarding to use the GUI on the host machine
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

# With root, create the mongodb folder and assign rights
RUN mkdir -p /opt/ros/mongodb_store && \
    chown developer: /opt/ros/mongodb_store && \
    chmod u+w /opt/ros/mongodb_store

WORKDIR /home/developer

#NOT WORKING IN DOCKERFILE
#CMD source /opt/ros/indigo/setup.bash && \
#    mkdir -p ./catkin_ws/src && \
#    cd ./catkin_ws &&\
#    catkin_make
    
# Change user and set up the environment variables
USER developer

ENV HOME /home/developer
# Set up Husky simulator for the use case
ENV HUSKY_GAZEBO_DESCRIPTION=/opt/ros/indigo/share/husky_description/urdf/description.gazebo.xacro



