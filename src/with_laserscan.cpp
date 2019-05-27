#include "with_laserscan.h"

Planner::Planner(Node start, Node goal)
{
    // Node s(0.0, 0.0);
    // Node g(3.0, 0.0);

    // ros::Subscriber scan_sub = n.subscribe("scan", 100, &Planner::scan_cb, this);
    // ros::Publisher nav_pub = n.advertise<nav_msgs::Path>("path", 100);

    this->s = start;
    this->g = goal;

    counter = 0;
    // this->rrt = rrt_ip;

    vector<Node> obstacles;
    for(int i=0;i<360;i++)
    {
        Node ob(999.0, 999.0);
        obstacles.push_back(ob);
    }

    Node cur_pos_con(robot_x, robot_y);
    RRT rrt(s, g, obstacles, -1.0, -4.0, 6.0, 4.0, 0.1, 500, 0.01);

    path = rrt.planning();
}

bool Planner::collision_check(Node node, vector<Node> obstacles)  // Return true if no obstacle
{
    float dist;
    // cout << "New point" << endl;
    for(int i=0;i<obstacles.size();i++)
    {
        // obstacles[i].print();
        // cout << (node.x - obstacles[i].x) << " " << 
        dist = sqrt(pow((node.x - obstacles[i].x), 2) + pow((node.y - obstacles[i].y), 2));
        // cout << dist << endl;
        if(dist<=OBSTACLE_THRESHOLD){
            return false;
        }
    }
    return true;

}

bool Planner::line_check(Node start, Node end, vector<Node> obstacle_list)  // Return true if no obstacle
{
    // cout << "Checking for: " << start.x << "," << start.y << " and " << end.x << "," << end.y << endl;
    float dist = sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
    float theta = atan2((end.y-start.y), (end.x-start.x));
    float min_jump = 0.01;
    int iterations = (int)(dist/min_jump);
    float st_x;
    float st_y;
    Node st_newnode;
    for(int i=0;i<iterations;i++)
    {
        st_x = start.x + min_jump*i*cos(theta);
        st_y = start.y + min_jump*i*sin(theta);
        st_newnode.x = st_x;
        st_newnode.y = st_y;
        // st_newnode.print();

        if(!Planner::collision_check(st_newnode, obstacle_list)) return false;
    }
    return true;
}

void Planner::odom_cb(const nav_msgs::Odometry& msg)
{
    robot_x = msg.pose.pose.position.x;
    robot_y = msg.pose.pose.position.y;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    robot_yaw = yaw;
    
}

float Planner::get_distance(vector<Node> path)
{
    float distance = 0;
    for(int i=1;i<path.size();i++)
    {
        distance += sqrt(pow((path[i].x - path[i-1].x), 2) + pow((path[i].y - path[i-1].y), 2));
    }

    return distance;

}

void Planner::scan_cb(const sensor_msgs::LaserScan& msg)
{
    float x_temp, y_temp;
    bool reached = false;
    
    bool path_changed = false; 
    bool checker = true;
    Node ob;

    vector<Node> obstacles;
    vector<Node> new_path;
    for(int i=0;i<360;i++)
    {
        
        
        if(msg.ranges[i]<3.4)
        {
            ob.x = msg.ranges[i]*cos((((double)i)*M_PI/180.0) + robot_yaw) + robot_x;
            ob.y = msg.ranges[i]*sin((((double)i)*M_PI/180.0) + robot_yaw) + robot_y;
            obstacles.push_back(ob);
            
        }
    }
    // cout << endl;


    /*******************************************/
    /* PLANNING ONLY IF PATH HAS OBSTACLES */
    checker = true;
    Node curr_pos(robot_x, robot_y);
    for(int i=1;i<2;i++)
    {
        for(int j=0;j<obstacles.size();j++)
        {

            if(!Planner::line_check(path[path.size()-2-counter], curr_pos, obstacles))
            {
                // cout << "obstacle" << endl;
                checker = false;
                break;
            }
        }
        if(!checker)
        {
            break;
        }
    }
    if(!checker)
    {
        RRT rrt(curr_pos, g, obstacles, -1.0, -4.0, 6.0, 4.0, 0.1, 500, 0.01);
        // RRT rrt(curr_pos, path[path.size()-2-counter], obstacles, -1.0, -4.0, 6.0, 4.0, 0.1, 500, 0.01);
        // new_path = path;
        path = rrt.planning();
        path_changed = true;
    } 
    // else
    // {
    //     Node curr_pos(robot_x, robot_y);
    //     RRT rrt(curr_pos, g, obstacles, -1.0, -2.0, 4.0, 2.0, 0.1, 500, 0.01);
    //     new_path = rrt.planning();
    //     if(get_distance(new_path) < get_distance(path))
    //     {
    //         path = new_path;
    //         path_changed = true;
    //     }
    // }
    
    /* *************************************** */


    
    

    cout << "path:" << endl;

    for (int i=0;i<path.size();i++)
    {
        cout << "(" << path[i].x << ", " << path[i].y << ") - ";
    }
    cout << endl;

    geometry_msgs::PoseStamped path_point;
    vector<geometry_msgs::PoseStamped> poses;
    for (int i=0;i<path.size();i++)
    {
        path_point.pose.position.x = path[i].x;
        path_point.pose.position.y = path[i].y;
        path_point.header.frame_id = "odom";
        poses.push_back(path_point);
    }

    nav_path.poses = poses;
    nav_path.header.frame_id = "odom";
    
    nav_pub.publish(nav_path);

    if(!path_changed)
    {
        float dist = sqrt(pow((robot_x - path[path.size()-2-counter].x), 2) + pow((robot_y - path[path.size()-2-counter].y), 2));
        cout << "Path point: " << path[path.size()-2-counter].x << " " << path[path.size()-2-counter].y << endl;
        cout << "pose: " << robot_x << " " << robot_y << endl;
        if(dist <= REACHED_THRESHOLD)
        {
            if(counter<path.size()-1) counter++;
            else 
            {
                cout << "Goal Reached" << endl;
                reached = true;
                exit(EXIT_FAILURE);
            }
        }
        cout << "dist: " << dist << endl;
    }
    else counter = 0;

    cout << "counter: " << counter << " path_size: " << path.size() << endl;
    std_msgs::Float32MultiArray point_pub;
    if(!reached)
    {
        point_pub.data.push_back(path[path.size()-2-counter].x);
        point_pub.data.push_back(path[path.size()-2-counter].y);
    }
    else
    {
        point_pub.data.push_back(g.x);
        point_pub.data.push_back(g.y);
    }

    path_pub.publish(point_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");

    float xs, ys, xg, yg;

    cout << "enter start:" << endl;
    cout << "x: ";
    cin >> xs;
    cout << "y: ";
    cin >> ys;

    cout << "enter goal:" << endl;
    cout << "x: ";
    cin >> xg;
    cout << "y: ";
    cin >> yg;

    Node s(xs, ys);
    Node g(xg, yg);

    Planner pl(s, g);
    ros::spin();

}
