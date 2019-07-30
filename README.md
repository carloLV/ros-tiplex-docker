# ROS-TiPlEx on docker
This folder contains the settings to create a docker image of ROS-TiPlEx, 
the engineering tool that helps robotic experts and planning experts to communicate and cooperate.


## Installation
You need [Docker](https://www.docker.com/) installed in order to run this demo.    
Please note that the installation currently works only on Linux OS. The arguments  `-e` and `-v` should be changed according to your machine to connect to the X11-server.

## Run the docker
Open a terminal we'll call **A** and clone this repo in your local machine, then navigate inside the repo and build the *docker image*.
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
       -p 9000:9000 \
       -p 9090:9090 \
       rostiplex:latest
```
In a new terminal **B** we'll create a new docker connection for later use.
First run `docker ps` and switch `docker_name` with the name of the active docker related to this image
```
docker exec -it docker_name /bin/bash
```

## Create ROS working environment
When you are logged in the docker, on terminal **A**, run:
```
/app/env-entrypoint.sh 
```
This script creates an environment for ROS and copies some data inside it. Then, it builds the environment with the **catkin_make** command.
You should see on console the output of the building process.

In the end let's start of all the nodes that partecipate to the demo, running the dedicated script:
```
./start_nodes.sh
```

## Run the Demo
On terminal **B** run the *python server* that will forward the ROS information.     
```
cd /app/planning_interface    
python -m SimpleHTTPServer 9000
```

Now write in the URL search of your browser `localhost:9000` and you will se the data in the docker. If something is not working, you should wait for all the nodes to start passing data.
From this point on you can start to use the tool.

#### Known issue
RVIz node is not perfectly working during the first run. If you close all the nodes and re-run the script `./start_nodes.sh` a second time, even RVIz will be ok.
It seems that the first time it runs, RVIz doesn't find the variable `HUSKY_GAZEBO_DESCRIPTION`.

For details on ROS-TiPlex visit the [wiki](https://github.com/carloLV/ROS-TiPlEx/wiki)
