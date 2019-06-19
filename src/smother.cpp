#include <iostream>
#include <cmath>
#include <vector>
#include "node.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"

#define min_jump 0.01 
vector<Node> smoother(vector<Node> path, Node start, Node end)
{
    vector<Node> straight_line;

    float dist = sqrt((start.x - end.x)*(start.x - end.x) + (start.y - end.y)*(start.y - end.y));
    float theta = atan2((end.y-start.y), (end.x-start.x));
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

        straight_line.push_back(st_newnode);
    }
    
}