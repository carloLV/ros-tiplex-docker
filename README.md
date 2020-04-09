# ROS-TiPlEx on docker
This folder contains the settings to create a docker image of ROS-TiPlEx,
the engineering tool that helps robotic experts and planning experts to communicate and cooperate.

## Installation
You need [Docker](https://www.docker.com/) installed in order to run this demo.    
**Please note that the installation currently works only on Linux OS**. The arguments  `-e` and `-v` should be changed according to your machine to connect to the X11-server.

## Run the docker
Open a terminal and clone this repo in your local machine, then navigate inside the repo and build the *docker image*.
```
git clone https://github.com/carloLV/ros-tiplex-docker.git
cd ros-tiplex-docker
docker build -t rostiplex:latest .
```
The image will be also available on the docker hub for download but so far this is the only way to get it.

Now you can run the docker and start working on it. Remove the `--rm` argument to the command below if you don't want to destroy the docker on its exit.     

```
docker run -it --rm \
       -e DISPLAY=unix$DISPLAY \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v "$(pwd)"/data:/app \
       -p 9000:9000 \
       -p 9090:9090 \
       rostiplex:latest
```

## Create ROS working environment
When you are logged in the docker, run:
```
/app/env-entrypoint.sh
```
This script creates an environment for ROS and copies some data inside it. Then, it builds the environment with the **catkin_make** command.
You should see on console the output of the building process.

## Run the Demo
To test the tasks of the roboticist, let's start all the nodes that partecipate to the process, running the dedicated script:
```
./start_node_robotic.sh
```
If you want to run the demo for the planner execute this script.
Note that for the planner, you need some data already existing in the MongoDB. You can install the dump available in the folder `message_store` to test
how the planner side implementation works.
```
./start_node_planner.sh
```
Both the `start_node_*.sh` will also run a Python server to allow access from the GUI

After running one of the two script mentioned above, write in the URL search of your browser `localhost:9000` and you will see the data in the docker.
From this point on you can start to use the tool.

#### Known issues
1. RVIz node is not perfectly working during the first run. If you close all the nodes and re-run the script `./start_node_robotic.sh` a second time, even RVIz will be ok.
It seems that the first time it runs, RVIz doesn't find the variable `HUSKY_GAZEBO_DESCRIPTION`.
1. When closing all the xterm of he nodes, sometimes the one related to MongoDB can not be closed. You should kill the related process instead.

For details on ROS-TiPlex visit the [wiki](https://github.com/carloLV/ROS-TiPlEx/wiki)
