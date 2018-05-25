# RoboND-Navigation-Project

## Projet Setup
### Step1: Create a Workspace
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```

### Step2: Add a Package
#### 0. Preparation
```
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-navigation
$ cd src
```

#### 1. gmapping
```
$ git clone https://github.com/ros-perception/slam_gmapping
$ rosdep -i install gmapping
```

#### 2. turtlebot_teleop
```
$ git clone https://github.com/turtlebot/turtlebot
```

#### 3. turtlebot_rviz_launchers
```
$ git clone https://github.com/turtlebot/turtlebot_interactions
```

#### 4. turtlebot_gazebo
```
$ git clone https://github.com/turtlebot/turtlebot_simulator
```

#### 5. build
```
$ cd ../
$ catkin_make
```
