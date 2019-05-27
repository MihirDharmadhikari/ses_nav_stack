# ses_nav_stack

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
