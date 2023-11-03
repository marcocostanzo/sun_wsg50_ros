# WSG50/32 for ROS1

## Description
This repo allows you to communicate and control the WSG50 gripper on ROS1 noetic. A docker container is provided to allow you to use it also in ROS2 by using ros_bridge.

### Using 
If you want to use the packages on your local machine follow the instruction in [local setup](#local-setup), otherwise you can use a docker container as in [docker setup](#docker-setup).

## Local setup
Follow the steps below to set up the package on your local machine. 

1. **Add the repository to your ROS workspace**
    ```bash
    cd ~/my_ros_ws/src
    git clone https://github.com/sara9915/uclv_wsg50_ros.git
    ```

2. **Install packages dependencies**
   
    The packages listed in `https.rosinstall` have to be installed. You can use `wstool` as follows:
   ```bash
    # In the src of your ros ws
    wstool init #if not already initialized
    wstool merge https://raw.githubusercontent.com/sara9915/uclv_wsg50_ros/main/https.rosinstall
    wstool update
    ```
   
    If you prefer to clone via ssh, merge with
    ```bash
    wstool merge https://raw.githubusercontent.com/sara9915/uclv_wsg50_ros/main/ssh.rosinstall
    ```
   
4. **Build**
    ```bash
    cd ~/my_ros_ws
    catkin build 
    ```
5. **Run the node**
    Before launching node, see all the available parameters:
    ```bash
    roslaunch sun_wsg50_driver wsg50_tcp_script.launch --ros-args
    ```
    and make sure to set your gripper ip to establish the communication:
   ```bash
    roslaunch sun_wsg50_driver wsg50_tcp_script.launch gripper_ip:="<your_gripper_ip>"
    ``` 
    
## Docker setup for ROS2 
If you need to use this package in ROS2, you can use the ros_bridge package. First, create the docker image:
```bash
docker build -t ros_noetic:wsg https://raw.githubusercontent.com/sara9915/uclv_wsg50_ros/main/Dockerfile
```
Then, create the docker container: 
```bash
docker run -it --name wsg_container ros_bridge:wsg
```
Now, you have a container with ROS1 Noetic, ROS2 Foxy and ros1_bridge installed. Also this repo and its dependecies are build into the container. 
To run the node and establish the communication on ROS2 follows these steps. Otherwise, you can simple run the nodes in ROS1.

1. **Launch the ROS1 node**
   ```bash
   docker run -it --name wsg_container ros_bridge:wsg
   source /opt/ros/noetic/setup.bash 
   roslaunch sun_wsg50_driver wsg50_tcp_script.launch gripper_ip:="<your_gripper_ip>"
    ```
2. **Launch the ROS bridge**

   In a new terminal, enter in the previous container:
   ```bash
   docker exec wsg_container /bin/bash
    ```
   Then, source the setup.bash to run ros2 commands:
   ```bash
   source /opt/ros/foxy/setup.bash 
    ```
   Now, launch the ros bridge:
   ```bash
    ros2 run ros1_bridge dynamic_bridge --bridge-all-topics 
    ```
    Run ```bash ros2 run ros1_bridge dynamic_bridge --help``` to see the available options for the bridge (ROS1->ROS2, or ROS2->ROS1, or ROS1<->ROS2).

   Now in a new terminal, you are able to see all the ROS1 topics from ROS2:
      ```bash
    ros2 topics list
    ```
