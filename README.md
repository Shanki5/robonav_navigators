# robonav_navigators

## Workspace Setup

Create a workspace for building all the ros packages

```mkdir -p ~/ros_hackathon_ws/src```

Initialize the workspace as a catkin workspace

`cd ~/ros_hackathon_ws && catkin init`

Clone all the related packages. Feel free to replace the ssh links if youk've setup ssh on your machine.

`cd ~/ros_hackathon_ws/src && git clone https://github.com/fetchrobotics/fetch_ros.git`

`git clone https://github.com/fetchrobotics/fetch_gazebo.git`

`git clone https://github.com/ros-drivers/rgbd_launch.git`

`git clone https://github.com/Shanki5/robonav_navigators.git`

Install all dependencies required by the packages

`sudo rosdep install --from-paths src --ignore-src -r -y`

Build all the packages

`catkin build`
