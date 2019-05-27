#include <iostream>
#include <stdlib.h>
#include <cmath>
#include "rrt.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
// #include "LinearMath/btMatrix3x3.h"

#define REACHED_THRESHOLD 0.05

class Planner
{
    private:
        Node s;
        Node g;

        float robot_x;
        float robot_y;
        float robot_yaw;

        int counter;

        ros::NodeHandle n;

        ros::Subscriber scan_sub = n.subscribe("scan", 100, &Planner::scan_cb, this);
        ros::Subscriber odom_sub = n.subscribe("odom", 100, &Planner::odom_cb, this);
        ros::Publisher nav_pub = n.advertise<nav_msgs::Path>("path", 100);
        ros::Publisher path_pub = n.advertise<std_msgs::Float32MultiArray>("final_path", 100);

        vector<Node> path;
        nav_msgs::Path nav_path;
        float temp_obstacle[2];

        RRT rrt;
        // RRT rrt(s, g, obstacles, -1.0, -2.0, 4.0, 2.0, 0.1, 500, 0.01);
    public:
        Planner(Node, Node);
        bool line_check(Node, Node, vector<Node>);
        bool collision_check(Node, vector<Node>);
        void scan_cb(const sensor_msgs::LaserScan&);
        void odom_cb(const nav_msgs::Odometry&);
        float get_distance(vector<Node>);
};