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
```bash
rosrun <name of package> planner
```
