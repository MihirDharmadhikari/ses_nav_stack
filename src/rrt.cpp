#include "rrt.h"

int get_min_index(float *, int);
// {
//     int i=0;
//     int id=0;
//     float min;
//     min = arr[0];
//     for(i=1;i<n;i++)
//     {
//         if(arr[i]<=min) 
//         {
//             min = arr[i];
//             id = i;
//         }
//     }
//     return id;
// }

RRT::RRT(){};

RRT::RRT(Node start, Node goal, vector<Node> obstacles, float minx=-5.0, float miny=-5.0, float maxx=5.0, float maxy=5.0, float expnddst=0.1, int max_iter=500, float min_jump=0.01)
{
    this->start = start;
    this->goal = goal;
    this->minx = minx;
    this->miny = miny;
    this->maxx = maxx;
    this->maxy = maxy;
    this->expand_distance = expnddst;
    this->max_iter = max_iter;
    this->min_jump = min_jump;
    this->obstacles = obstacles;
    this->goal_sample_rate = 15;
    this->conv_error = 9999.0;
    prev_path.push_back(goal);
    prev_path.push_back(start);

}

bool RRT::collision_check(Node node)  // Return true if no obstacle
{
    float dist;
    for(int i=0;i<obstacles.size();i++)
    {
        dist = sqrt(pow((node.x - obstacles[i].x), 2) + pow((node.y - obstacles[i].y), 2));
        if(dist<=OBSTACLE_THRESHOLD){
            return false;
        }
    }
    return true;

}

void RRT::update_obstacles(vector<Node> obstacle_list)
{
    this->obstacles = obstacle_list;

    for(int i=0;i<(this->obstacles).size();i++)
    {
        (this->obstacles)[i].x = obstacle_list[i].x;
        (this->obstacles)[i].y = obstacle_list[i].y;
    }
    for(int i=0;i<(this->obstacles).size();i++) std::cout << (this->obstacles)[i].x << " " << (this->obstacles)[i].x << endl;
}

bool RRT::line_check(Node start, Node end)  // Return true if no obstacle
{
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

        if(!RRT::collision_check(st_newnode)) return false;
    }
    return true;
}

int RRT::get_nearest_index(float * rnd, vector<Node> nodelist)
{
    float dlist[nodelist.size()];
    float min_dist;
    for(int i=0;i<nodelist.size();i++)
    {
        *(dlist+i) = (nodelist[i].x-rnd[0])*(nodelist[i].x-rnd[0]) + (nodelist[i].y-rnd[1])*(nodelist[i].y-rnd[1]);
    }
    return get_min_index(dlist, nodelist.size());

}

float RRT::calculate_path_length(vector<Node> path)
{
    float path_length = 0;
    for(int i=1;i<path.size();i++)
    {
        path_length += (float)sqrt(pow((double)(path[i].x - path[i-1].x), 2) + pow((double)(path[i].y - path[i-1].y), 2));
    }
    return path_length;
}

vector<Node> RRT::planning()
{
    vector<Node> nodelist;
    // vector<float *> path;
    vector<Node> path;
    vector<Node> zero_path;
    vector<Node> final_path;
    nodelist.push_back(start);
    int nind;
    float rnd[2];
    Node nearestNode;
    float theta;
    float d;
    float temp[2];
    int last_index;
    int iteration_cnt = 0;
    float conv_dist = 0.1;

    srand(time(NULL));

    while (true)
    {
        if(RRT::line_check(start, goal))
        {
            nodelist.push_back(goal);
            break;
        }
        if(rand()%100 > goal_sample_rate) 
        {
            // rnd[0] = (float)((rand()%((int)(maxx-minx)*1000) + minx*1000))/1000.0;
            // rnd[1] = (float)((rand()%((int)(maxy-miny)*1000) + miny*1000))/1000.0;

            // for(int iterator=0;iterator<iteration_cnt;iterator++)
            // {
            rnd[0] = (float)((rand()%((int)(maxx-minx)*1000) + minx*1000))/1000.0;
            rnd[1] = (float)((rand()%((int)(maxy-miny)*1000) + miny*1000))/1000.0;     
            // }
            
            // rnd[0] = (float)((rand()%((int)(maxx-minx)*1000) + minx*1000))/1000.0;
            // rnd[1] = (float)((rand()%((int)(maxy-miny)*1000) + miny*1000))/1000.0; 

        }
        else 
        {
            rnd[0] = goal.x;
            rnd[1] = goal.y;
        }

        // std::cout << rnd[0] << " | " << rnd[1] << endl;

        nind = RRT::get_nearest_index(rnd, nodelist);

        Node nearestNode(nodelist[nind].x, nodelist[nind].y);
        // nearestNode = nodelist[nind];
        theta = atan2(rnd[1]-nearestNode.y, rnd[0]-nearestNode.x);

        nearestNode.x += expand_distance*cos(theta);
        nearestNode.y += expand_distance*sin(theta);
        nearestNode.parent = nind;

        // std::cout << nearestNode.x << " " << nearestNode.y << " " << nearestNode.parent << endl;

        if(!RRT::collision_check(nearestNode)) continue;
        nodelist.push_back(nearestNode);
        // std::cout << nodelist.size() << endl;

        d = sqrt(pow((double)(nearestNode.x-goal.x),2) + pow((double)(nearestNode.y-goal.y),2));

        if(d <= expand_distance) 
        {   
            // std::cout << "goal" << endl;
            break;
        }
            
        else
        {
            if(RRT::line_check(nearestNode, goal)) break;
        }
        iteration_cnt++;
        if(iteration_cnt > max_iter)
        {
            std::cout << "Max iterations done. Path not found" << endl;
            return zero_path;
        }
    }

    path.push_back(goal);
    last_index = (nodelist.size()) - 1;

    int j = 1;
    while(nodelist[last_index].parent != -1)
    {

        path.push_back(nodelist[last_index]);

        // std::cout << path[j].x << " | " << path[j].y << endl;
        last_index = nodelist[last_index].parent;
        j++;
    }

    float temp3[2] = {start.x, start.y};
    path.push_back(start);

    // std::cout << "Path: " << endl;

    // for(int i=0;i<path.size();i++) std::cout << path[i].x << " | " << path[i].y << endl;

    final_path.push_back(goal);
    for(int i=1;i<path.size();i++)
    {
        if(RRT::line_check(final_path[final_path.size()-1], path[i]));
        else final_path.push_back(path[i-1]);
        // std::cout << i << endl;
    }
    final_path.push_back(start);

    // std::cout << "Final Path: " << endl;

    // for(int i=0;i<final_path.size();i++) std::cout << final_path[i].x << " | " << final_path[i].y << " ";
    // std::cout << endl;
    // do
    // std::cout << "Conv error: " << conv_error << endl;
    // while(conv_error > conv_dist)
    for(int c=0;c<2;c++)
    {
        // std::cout << "INSIDE WHILE LOOP" << endl;
        // std::cout << "Conv error beg: " << conv_error << endl;
        for(int i=1;i<final_path.size()-1;)
        {
            Node n1, n2, n1_prev, n2_prev;
            bool path_smoothened = false;
            float min_jump = 0.01;
            float d1, d2;
            d1 = sqrt((final_path[i].x - final_path[i-1].x)*(final_path[i].x - final_path[i-1].x) + (final_path[i].y - final_path[i-1].y)*(final_path[i].y - final_path[i-1].y));
            d2 = sqrt((final_path[i+1].x - final_path[i].x)*(final_path[i+1].x - final_path[i].x) + (final_path[i+1].y - final_path[i].y)*(final_path[i+1].y - final_path[i].y));
            float theta1 = atan2((final_path[i].y-final_path[i-1].y), (final_path[i].x-final_path[i-1].x));
            float theta2 = atan2((final_path[i+1].y-final_path[i].y), (final_path[i+1].x-final_path[i].x));
            int i1, i2, iter;
            i1 = (int)(d1/min_jump);
            i2 = (int)(d2/min_jump);
            if(i1<i2) iter = i1;
            else iter = i2;
            // std::cout << "iter: "<< iter << endl;
            for(int j=1;j<iter;j++)
            {
                n1.x = final_path[i].x - min_jump*j*cos(theta1);
                n1.y = final_path[i].y - min_jump*j*sin(theta1);

                n2.x = final_path[i].x + min_jump*j*cos(theta2);
                n2.y = final_path[i].y + min_jump*j*sin(theta2);

                // std::cout << "n1: ";
                // n1.print();
                // std::cout << "n2: ";
                // n2.print();

                if(!(RRT::line_check(n1, n2)))
                {
                    // std::cout << "n1 prev: ";
                    // n1_prev.print();
                    // std::cout << "n2 prev: ";
                    // n2_prev.print();
                    final_path.insert((final_path.begin()+i), n1_prev);
                    final_path.insert((final_path.begin()+i+2), n2_prev);
                    final_path.erase(final_path.begin()+i+1);
                    i = i+3;
                    // std::cout << "smoothened" << endl;
                    path_smoothened = true;
                    break;
                }
                
                n1_prev = n1;
                n2_prev = n2;
            }
            if (!path_smoothened)
            {
                // std::cout << "Not smoothened " << i << endl;
                // std::cout << final_path.size()-1 << endl;
                final_path.insert((final_path.begin()+i), n1_prev);
                final_path.insert((final_path.begin()+i+2), n2_prev);
                final_path.erase(final_path.begin()+i+1);
                i = i+3;
            } 
            // std::cout << "smoothening" << endl;
        }
        // conv_error = fabs(RRT::calculate_path_length(prev_path) - RRT::calculate_path_length(final_path));
        // std::cout << "Conv error end: " << conv_error << endl;
        // prev_path = final_path;
    }

    // std::cout << "Final Path: " << endl;

    // for(int i=0;i<final_path.size();i++) std::cout << final_path[i].x << ", " << final_path[i].y << " | ";
    // std::cout << endl;

    return final_path;
}

int get_min_index(float * arr, int n)
{
    int i=0;
    int id=0;
    float min;
    min = arr[0];
    for(i=1;i<n;i++)
    {
        if(arr[i]<=min) 
        {
            min = arr[i];
            id = i;
        }
    }
    return id;
}