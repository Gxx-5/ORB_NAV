#include<ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

using namespace std;

class CostCube
{
typedef vector<vector<vector<float>>> Cube;

public:
        CostCube(double side_length,double resolution);
        Cube getCostCube(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose);

private:
        double side_length;
        double resoluion;
};