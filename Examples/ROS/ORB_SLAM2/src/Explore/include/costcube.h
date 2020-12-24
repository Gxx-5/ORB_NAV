#include<ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

using namespace std;

class CostCube
{
public:
        typedef vector<vector<vector<float>>> Cube;
        // typedef vector<vector<float>> cube_slice;

        CostCube(double len,double res);
        Cube getCostCube(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose);

private:
        double side_length;
        double resoluion;
        int cell_num;
};