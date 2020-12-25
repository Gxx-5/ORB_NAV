#include<ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>

using namespace std;

class CostCube
{
public:
        typedef vector<vector<vector<float>>> Cube;
        // typedef vector<vector<float>> cube_slice;

        CostCube(double len,double res);
        Cube getCostCube(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose);
        void processMapPts(const std::vector<geometry_msgs::Point> &pts, unsigned int n_pts,
				   unsigned int start_id, const geometry_msgs::Point &cam_pos);
        void Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,
				  cv::Mat &visited,const geometry_msgs::Point &cam_pos);
private:
        double side_length;
        double resoluion;
        int voxel_n;
        int size[3];
        cv::Mat occupied_counter, visit_counter;
};