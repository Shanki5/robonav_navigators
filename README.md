# robonav_navigators

## Workspace Setup

Create a workspace for building all the ros packages

```
mkdir -p ~/ros_hackathon_ws/src
```

Initialize the workspace as a catkin workspace

```
cd ~/ros_hackathon_ws && catkin init
```

Clone all the related packages. Feel free to replace the ssh links if youk've setup ssh on your machine.

```
cd ~/ros_hackathon_ws/src && git clone https://github.com/fetchrobotics/fetch_ros.git
```

```
git clone https://github.com/fetchrobotics/fetch_gazebo.git
```

```
git clone https://github.com/ros-drivers/rgbd_launch.git
```

```
git clone https://github.com/Shanki5/robonav_navigators.git
```

Install all dependencies required by the packages

```
cd ~/ros_hackathon_ws && sudo rosdep install --from-paths src --ignore-src -r -y
```

Build all the packages

```
catkin build
```

## How to run the simulation and packages

You will have to source the workspace each time you build a new package or if you haven't sourced in the terminal you are working on

```
source ~/ros_hackathon_ws/devel/setup.bash
```

Launch the simulation

```
roslaunch robonav_navigators maze_sim.launch
```

Run gmapping. This also starts move_base

```
roslaunch robonav_navigators gmapping.launch
```

To run scripts using `rosrun robonav_navigators [script name]`. Note that you will have to add executable permission to the script before being able to run it. Do that using `chmod +x [filename]` . Use double tab to see the list of all the scripts in each package for ease of use.
