# ros-tiplex-docker
This folder contains the settings to create a docker image of ROS-TiPlEx, 
the engineering tool that lets robotic expert and planning experts to communicate and cooperate.

# Bring upd the environment
1. First of all, clone this repo in your local machine, then navigate inside the repo and run:      
`docker built -t rostiplex:latest .`
The image will be also available on the docker hub for download but so far this is the only way to get it.

2. Once your image is built, you can start the docker and start working on it. Run this command to run the docker. Add the `--rm` argument if you want to destroy the docker on exit. To simply run the docker:     

`docker run -it -e DISPLAY=unix$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -p 9000:9000 -p 9090:9090 rostiplex:latest`

3. When you are logged in the docker, there is some prepwork to set upp all the environment and be able to run the system. First of all run: `/app/env-entrypoint.sh `.     
You should see the building process of **catkin_make** in action.    

4. Now the setup is finished, and you can start the nodes running the dedicated script, with this command: `./start_nodes.sh`

5. The last step is to run the python server in order to be able to use the provided interface. To run the server start opening a new terminal and connecting to the docker using: `docker exec -it *docker_name* /bin/bash`.
You'll be logged in. Now run this two commands:     
`cd /app/planning_interface`     
`python -m SimpleHTTPServer 9000`.

Now in your browser, in the URL search bar write `localhost:9000` and you will se the data in the docker. IF something is not working, maybe you should wait for all the nodes to start passing data.
From this point on you can start to use the tool.

## Known issue
RVIZ node is not perfectly working during the first run. If you close all the nodes and re-run the script `./start_nodes.sh` a second time, even RVIZ will be ok.
