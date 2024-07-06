# SMART DRIVE PROJECT  <br>
## ROS2 Jazzy Jalisco + Gazebo Harmonic + Ubuntu 24.04 Noble

This project aims to develop a high-fidelity digital twin of a 4WD vehicle, where each wheel's torque is individually controlled and steering is achieved through a hinge-based system. This innovative approach eliminates reliance on standard plugins, allowing the vehicle's movement and trajectory to be determined solely by its interaction with the terrain.

To demonstrate the capabilities of this digital twin, I've created a simple test track environment, marked by cones, where the vehicle's performance can be evaluated under various conditions.

## How to download [IMPORTANT]:
Since it use a submodules that contain the models of the camera and lidar, clone this repo using `git clone --recurse-submodules <this-repo-url>`

***ACHTUNG!*** Docker is needed to be installed on your system to make it works, to install it I suggest `https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-22-04`

## How to build the image and start the container
1) Go inside the folder `scripts`
2) Create an image from the Dockerfile: `bash build-docker.sh`
3) Run the container: `bash start-sim.sh`
4) **Achtung!** Once inside, only for the first time **go to deps/ folder and launch `bash first_launch_script.sh`**: this will setup everything for you and configure all the dependencies. 
5) Now you are inside the container and everything is ready to launch


## How to use it:
1) At every pc reboot you need to do  `docker start psd_container`
2) After that you can access the container with `docker exec -it psd_container /bin/bash`
3) To start the sim you need to launch it from `psd_ws/startup/` folder this script `bash sim_launch.sh`
3) CTRL + D if you want to exit the container

## First run of the container:
To be sure that everything is setup well, check the README in the ***deps/*** folder
