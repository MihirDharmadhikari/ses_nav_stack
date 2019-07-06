#include <vector>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <time.h>
#include "node.h"

#define OBSTACLE_THRESHOLD 0.15
#define G_THRESHOLD 1

using namespace std;


class RRT
{
    private:
        Node start;
        Node goal;
        
        // Define the area of sampling
        float minx;
        float miny;
        float maxx;
        float maxy;
        int goal_sample_rate;
        float conv_error;

        float expand_distance;
        int max_iter;
        float min_jump;  // When checking intersection of a line with the obstacles

        vector<Node> obstacles;
        vector<Node> prev_path;


    public:
        RRT();
        RRT(Node, Node, vector<Node>, float, float, float, float, float, int, float);

        bool collision_check(Node);  // True if no obstacle
        bool line_check(Node, Node);  // True if line does not hit the obstacles
        int get_nearest_index(float *, vector<Node>);
        vector<Node> planning();
        void update_obstacles(vector<Node>);
        float calculate_path_length(vector<Node>);
    
};