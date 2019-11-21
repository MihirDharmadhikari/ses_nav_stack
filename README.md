# ses_nav_stack
RRT based dynamic path planning package robots having 2D lidar system. The planner is currently tested on Turtlebot3 and used its odometry package for localization. 

# Dependancies:
- ROS kinetic on Ubuntu 16.04: http://wiki.ros.org/ROS/Installation
- Turtlebot3 software stack:
  ```bash
  sudo apt-get install ros-kinetic-turtlebot3*
  cd <catkin_ws>/src
  git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
  ```
# Running the C++ implementation:
Only the obstacle detection and path planning has been implemented in C++. The controller has to be used as it is from the scripts folder. This part is yet to be integrated with the Global planner.

Pull the repo.
Run the following commands to compile and run
Compile:
```bash
cd <catkin_ws>
catkin_make
source devel/setup.bash
```

Run:
Terminal 1:
```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
Add few obstacles in the environment.

Terminal 2:
```bash
rosrun <name of package> planner
```
Give the start as the current position of the bot and goal anypoint between x:[-1, 6], y:[-4, 4]

Terminal 3:
```bash
roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```
Add path msg into rviz from the add button on bottom left

Terminal 4:
```bash
cd <path to package>/scripts
python controller.py
```

Enjoy!
