#include "ros/ros.h"
#include<iostream>
#include<vector>
#include<math.h>
#include <ctime>
#include <cstdlib>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort
#include<cmath>
#include "tf/transformations.h"
#include "geomerty_msgs/Vector3.h"

using namespace std;

int main()
{
	
	float q {0.0, 0.0, 0.0, 1.0};

	geomerty_msgs::Vector3 a = tf::transformations::euler_from_quaternion(q);

	for(int i=0;i<3;i++) cout << a[i] << endl;
	

	return 0;
}