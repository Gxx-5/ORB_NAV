#include<ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>
#include<opencv2/viz.hpp>

using namespace std;

/*
*****************************************************************
*inflation_layer.h* 
ComputeCost base on distance between current cell and closest 
obstacle cell
*****************************************************************
  virtual inline unsigned char computeCost(double distance) const
  {
    unsigned char cost = 0;
    if (distance == 0)
      cost = LETHAL_OBSTACLE;
    else if (distance * resolution_ <= inscribed_radius_)
      cost = INSCRIBED_INFLATED_OBSTACLE;
    else
    {
      // make sure cost falls off by Euclidean distance
      double euclidean_distance = distance * resolution_;
      double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
      cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
    }
    return cost;
  }
*/

class CostCube
{
public:
        // typedef vector<vector<vector<float>>> Cube;
        // typedef vector<vector<float>> cube_slice;
        static const unsigned char NO_INFORMATION = 255;
        static const unsigned char LETHAL_OBSTACLE = 254;
        static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
        static const unsigned char FREE_SPACE = 0;
        cv::Mat map_prob;

        CostCube(double radius,double res);
        cv::Mat calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose);
        cv::Mat calCostCubeByDistance(vector<geometry_msgs::Point> map_points,geometry_msgs::Pose camera_pose);
        void processMapPts(const std::vector<geometry_msgs::Point> &pts, const geometry_msgs::Point &cam_pos,bool cal_occupied_only=false);
        void Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,
				  cv::Mat &visited,const geometry_msgs::Point &cam_pos,bool cal_occupied_only=false);
        float computeCostByDistance(const float distance);
        float dstFromVoxelToObstacle(vector<int> pos_id);

private:
        double filter_radius;
        double resoluion;
        int voxel_n;
        int size[3];
        cv::Mat occupied_counter, visit_counter;
        vector<vector<int>> occupied_ind;
        int free_thresh = 5;
        int occupied_thresh = 5;
        double inscribed_radius_ = 0.18;
        double cost_scaling_factor = 10.0;
};