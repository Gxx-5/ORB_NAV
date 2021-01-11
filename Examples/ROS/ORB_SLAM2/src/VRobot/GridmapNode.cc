#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iterator>
#include <ctime>
#include <vector>
#include <set>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;

const double PI = 3.14159265358979323846; /* pi */

// parameters
float resolution = 0.02;//resolution of map
float map_width = 10.0;
float map_height = 10.0;

float scale_factor = 1;//3
float resize_factor = 0.1;//5
float cloud_max_x = 60;//10;
float cloud_min_x = -60;//-10.0;
float cloud_max_z = 80;//16;
float cloud_min_z = -25;//-5;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
unsigned int use_local_counters = 0;
int visit_thresh = 0;

float thresh_diff = 0.01;
float upper_left_x = -1.5;
float upper_left_y = -2.5;


float map_rbound, map_lbound, map_ubound, map_bbound;
cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh; //, grid_map_thresh_resized, grid_map_proba;

float norm_factor_x, norm_factor_z;
int h, w;
unsigned int n_kf_received;
bool loop_closure_being_processed = false;
ros::Publisher pub_grid_map,pub_pose;
nav_msgs::OccupancyGrid grid_map_msg;

float kf_pos_x, kf_pos_y, kf_pos_z;
int kf_pos_grid_x = 0, kf_pos_grid_z = 0;

int g_camera_pos_grid_x = 0, g_camera_pos_grid_z = 0; //相机位置
int g_camera_ori_grid_x = 0, g_camera_ori_grid_z = 0; //相机方向位置
int g_target_x, g_target_z;
bool g_target_is_set = false;

std::string param_str;

vector<vector<double>> kf_plantVec(0, vector<double>(3)); //关键帧平面的世界坐标集

vector<double> bestplane(3, 0);				//平面拟合 模型方程z=ax+by+c 法向量为[a,b,-1], [0]a [1]b [2]c
vector<double> idealNormalVec = {0, -1, 0}; //理想的平面法向量
vector<vector<double>> rotationMatrix = {
	{1, 0, 0},
	{0, 1, 0},
	{0, 0, 1},
}; //旋转矩阵

geometry_msgs::PoseStamped curpose;


std::mutex g_gridMapMutex;
bool g_findpathRunning = true;

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void saveMap(unsigned int id = 0);
void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr &cur_camera_pose);
tf::Transform getmap2odom(const geometry_msgs::PoseStamped grid_pose);
void ptCallback(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts);
void parseParams(int argc, char **argv);
void printParams();
//void getMixMax(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose,
//			   geometry_msgs::Point &min_pt, geometry_msgs::Point &max_pt);
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
				  cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
				   unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
void getGridMap();

vector<double> CrossProduct(vector<double> &a, vector<double> &b);
double DotProduct(vector<double> &a, vector<double> &b);
double Normalize(vector<double> &v);
void PlaneFittingRansac(vector<vector<double>> &Vectors, vector<double> &bestplane);
vector<vector<double>> CalRotationMatrix(vector<double> &vectorBefore, vector<double> &vectorAfter);
vector<double> RotationAjust(vector<vector<double>> &r, vector<double> &v);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "VRobot");
	ros::start();

	printf("VRobot:Input %d params\n", argc - 1);
	parseParams(argc, argv);
	printParams();

	{
		const std::vector<float> params = {
			scale_factor, resize_factor, cloud_max_x, cloud_min_x,
			cloud_max_z, cloud_min_z, free_thresh, occupied_thresh,
			(float)use_local_counters, (float)visit_thresh};
		std::ostringstream oss;
		std::copy(params.cbegin(), params.cend(), ostream_iterator<float>(oss, "_"));
		param_str = oss.str();
	}

	map_rbound = map_width/2;
	map_lbound = -map_width/2;
	map_ubound = map_height/2;
	map_bbound = -map_height/2;
	printf("map_bound: %f, %f, %f, %f\n", map_ubound, map_bbound, map_lbound, map_rbound);

	w = map_width / resolution;
	h = map_height / resolution;
	printf("grid_size: (%d, %d)\n", w, h);
	n_kf_received = 0;

	global_occupied_counter.create(h, w, CV_32SC1);
	global_visit_counter.create(h, w, CV_32SC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	grid_map_msg.data.resize(h * w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = resolution;

	grid_map_int = cv::Mat(h, w, CV_8SC1, (char *)(grid_map_msg.data.data()));

	//grid_map_proba.create(h, w, CV_8UC1);

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	//grid_map_thresh_resized.create(h * resize_factor, w * resize_factor, CV_8UC1);
	//printf("output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32SC1);
	local_visit_counter.create(h, w, CV_32SC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	norm_factor_x = 1.0/resolution;//float(grid_res_x - 1) / float(map_rbound - map_lbound);
	norm_factor_z = 1.0/resolution;//float(grid_res_z - 1) / float(map_ubound - map_bbound);
	printf("norm_factor_x: %f\n", norm_factor_x);
	printf("norm_factor_z: %f\n", norm_factor_z);


	curpose.header.seq = 0;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id = "map";
	// 地图点需按分辨率映射
	curpose.pose.position.x =0;
	curpose.pose.position.y = 0;
	curpose.pose.position.z = 0;
	curpose.pose.orientation.x = 0;
	curpose.pose.orientation.y = 0;
	curpose.pose.orientation.z = 0;
	curpose.pose.orientation.w = 1;

	// ros::Rate loop_rate(100);
	
	ros::NodeHandle nodeHandler;
	ros::Subscriber sub_pts_and_pose = nodeHandler.subscribe("/pts_and_pose", 1000, ptCallback);
	ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("/all_kf_and_pts", 1000, loopClosingCallback);
	ros::Subscriber sub_cur_camera_pose = nodeHandler.subscribe("/cur_camera_pose", 1000, cameraPoseCallback);
	
	// ros::Subscriber sub_grid_pose = nodeHandler.subscribe("/grid_pose", 1000, gridPoseCallback);
	
	pub_grid_map = nodeHandler.advertise<nav_msgs::OccupancyGrid>("map", 1000);
	pub_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("grid_pose", 1);
	
	// cv::namedWindow("grid_map_thresh", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("grid_map_thresh", onMouseHandle);

	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(100);
	while(ros::ok()){
		ros::Time now = ros::Time::now();
		pub_pose.publish(curpose);
		tf::Transform map2odom = getmap2odom(curpose);
		broadcaster.sendTransform(tf::StampedTransform(map2odom,now,"map", "odom"));
		printf("[%lf] send transform from map to odom!\n",now.toSec());
		ros::spinOnce();
		loop_rate.sleep();
	}

	ros::shutdown();
	cv::destroyAllWindows();
	saveMap(time(NULL));

	g_findpathRunning = false;

	return 0;
}

void onMouseHandle(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		g_target_x = static_cast<int>(x / resize_factor);
		g_target_z = static_cast<int>((h-y) / resize_factor);
		if (g_target_x < 0 || g_target_x >= w ||
			g_target_z < 0 || g_target_z >= h)
		{
			g_target_is_set = false;
		}
		else
		{
			g_target_is_set = true;
		}
		//printf("onMouseHandle: Set target: %d, %d (Current: %d, %d)\n",
		//		int(g_target_x*resize_factor), int(g_target_z*resize_factor),
		//		int(g_camera_pos_grid_x*resize_factor), int(g_camera_pos_grid_z*resize_factor));
		break;
	}
}

//publish tranform from map to odom . 
tf::Transform getmap2odom(const geometry_msgs::PoseStamped grid_pose)
{
	tf::Transform map2odom;
	tf::TransformListener listener;
	try
    	{  
		ros::Time now = ros::Time(0);
		tf::Transform map2base = 
		tf::Transform(tf::Quaternion(grid_pose.pose.orientation.x, grid_pose.pose.orientation.y, grid_pose.pose.orientation.z, grid_pose.pose.orientation.w),
		   	 							tf::Vector3(grid_pose.pose.position.x, grid_pose.pose.position.y,grid_pose.pose.position.z));
		tf::StampedTransform odom2base;
		// printf("map2base,%f,%f,%f\n%f,%f,%f\n",map2base.getRotation()[0],map2base.getRotation()[1],map2base.getRotation()[2],map2base.getOrigin()[0],map2base.getOrigin()[1],map2base.getOrigin()[2]);									  
		listener.waitForTransform("odom","base_link",now,ros::Duration(2));
		listener.lookupTransform("/odom", "/base_link",now, odom2base);
		tf::Transform base2odom = odom2base.inverse();
		tf::Transform base2map = map2base.inverse();
		// printf("base2odom,%f,%f,%f\n%f,%f,%f\n",base2odom.getRotation()[0],base2odom.getRotation()[1],base2odom.getRotation()[2],base2odom.getOrigin()[0],base2odom.getOrigin()[1],base2odom.getOrigin()[2]);									  
		map2odom = base2map.inverse()*base2odom;
		return map2odom;
	}
	catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	// ros::Duration(1.0).sleep();
	}
	return map2odom;
}

void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr &cur_camera_pose)
{
	const geometry_msgs::Point &location = cur_camera_pose->position;
	const geometry_msgs::Quaternion &orientation = cur_camera_pose->orientation;

	//const float camera_pos_x = location.x * scale_factor;
	//const float camera_pos_z = location.z * scale_factor;

	vector<double> curr_pt_before = {location.x, location.y, location.z};
	auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before); //旋转调整
	const float camera_pos_x = curr_pt_after[0] * scale_factor;
	const float camera_pos_z = curr_pt_after[2] * scale_factor;

	Eigen::Quaterniond q = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).normalized();
	Eigen::Vector3d p1 = Eigen::Vector3d(0, 0, 1); //世界坐标系正前方向，z正前，x左，y下

	Eigen::Vector3d p2 = q * p1; //相机在世界坐标系的方向

	vector<double> p3 = {p2[0], p2[1], p2[2]};
	p3 = RotationAjust(rotationMatrix, p3); //旋转调整

	const int camera_pos_grid_x = int(floor((camera_pos_x - map_lbound) * norm_factor_x));
	const int camera_pos_grid_z = int(floor((camera_pos_z - map_bbound) * norm_factor_z));

	if (camera_pos_grid_x < 0 || camera_pos_grid_x >= w ||
		camera_pos_grid_z < 0 || camera_pos_grid_z >= h)
		return;

	g_camera_pos_grid_x = camera_pos_grid_x;
	g_camera_pos_grid_z = camera_pos_grid_z;

	g_camera_ori_grid_x = g_camera_pos_grid_x + p3[0] * 10;
	g_camera_ori_grid_z = g_camera_pos_grid_z + p3[2] * 10;

}

void saveMap(unsigned int id)
{

	cv::Mat grid_map_int_fliped, grid_map_thresh_fliped, grid_map_thresh_resized_fliped;
	{
		std::lock_guard<std::mutex> lock(g_gridMapMutex);
		cv::flip(grid_map_int, grid_map_int_fliped, 0);
		cv::flip(grid_map_thresh, grid_map_thresh_fliped, 0);
		//cv::flip(grid_map_thresh_resized, grid_map_thresh_resized_fliped, 0);
	}

	printf("saving maps with id: %u\n", id);
	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (id > 0)
	{
		cv::imwrite("results/grid_map_" + to_string(id) + "_" + param_str + ".png", grid_map_int_fliped);
		cv::imwrite("results/grid_map_thresh_" + to_string(id) + "_" + param_str + ".png", grid_map_thresh_fliped);
		//cv::imwrite("results/grid_map_thresh_resized_" + to_string(id) + "_" + param_str + ".png", grid_map_thresh_resized_fliped);
	}
	else
	{
		cv::imwrite("results/grid_map_" + param_str + ".png", grid_map_int_fliped);
		cv::imwrite("results/grid_map_thresh_" + param_str + ".png", grid_map_thresh_fliped);
		//cv::imwrite("results/grid_map_thresh_resized_" + param_str + ".png", grid_map_thresh_resized_fliped);
	}
}
void ptCallback(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose)
{
	// ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),pts_and_pose->header.seq);
	//if (pts_and_pose->header.seq==0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	printf("Received exit message\n");
	//	ros::shutdown();
	//	exit(0);
	//}
	//	if (!got_start_time) {
	//#ifdef COMPILEDWITHC11
	//		start_time = std::chrono::steady_clock::now();
	//#else
	//		start_time = std::chrono::monotonic_clock::now();
	//#endif
	//		got_start_time = true;
	//	}
	if (loop_closure_being_processed)
	{
		return;
	}

	updateGridMap(pts_and_pose);

	grid_map_msg.info.map_load_time = ros::Time::now();
	pub_grid_map.publish(grid_map_msg);
}
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (all_kf_and_pts->header.seq == 0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	ros::shutdown();
	//	exit(0);
	//}
	cout << "loopClosingCallback" << time(NULL) << endl;
	loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
}
/*
void getMixMax(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose,
			   geometry_msgs::Point &min_pt, geometry_msgs::Point &max_pt)
{

	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	for (unsigned int i = 0; i < pts_and_pose->poses.size(); ++i)
	{
		const geometry_msgs::Point &curr_pt = pts_and_pose->poses[i].position;
		if (curr_pt.x < min_pt.x)
		{
			min_pt.x = curr_pt.x;
		}
		if (curr_pt.y < min_pt.y)
		{
			min_pt.y = curr_pt.y;
		}
		if (curr_pt.z < min_pt.z)
		{
			min_pt.z = curr_pt.z;
		}

		if (curr_pt.x > max_pt.x)
		{
			max_pt.x = curr_pt.x;
		}
		if (curr_pt.y > max_pt.y)
		{
			max_pt.y = curr_pt.y;
		}
		if (curr_pt.z > max_pt.z)
		{
			max_pt.z = curr_pt.z;
		}
	}
}*/

void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
				  cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z)
{
	vector<double> curr_pt_before = {curr_pt.x, curr_pt.y, curr_pt.z};
	auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);

	float pt_pos_x = curr_pt_after[0] * scale_factor;
	float pt_pos_y = curr_pt_after[0] * scale_factor;
	float pt_pos_z = curr_pt_after[2] * scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - map_lbound) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - map_bbound) * norm_factor_z));

	//TODO jark
	//if(curr_pt.y > (kf_pos_y + 0.22))
	//	return;
	//if (pt_pos_y < (kf_pos_y - 1.2)) //高度过滤
	//	return;

	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(pt_pos_grid_z, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

	// Get all grid cell that the line between keyframe and map point pass through
	/*int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x)
	{
		if (steep)
		{
			++visited.at<int>(x, y);
		}
		else
		{
			++visited.at<int>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5)
		{
			y = y + ystep;
			error = error - 1.0;
		}
	}*/

	int x1 = kf_pos_grid_x;
	int y1 = kf_pos_grid_z;
	int x0 = pt_pos_grid_x;
	int y0 = pt_pos_grid_z;

	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;

	for (;;)
	{
		++visited.at<int>(y0, x0);
		if (x0 == x1 || y0 == y1)
			break;
		e2 = err;
		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
	}
	/* 
	来源：https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
	详解：https://blog.csdn.net/cjw_soledad/article/details/78886117 
	*/
}

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
				   unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z)
{
	unsigned int end_id = start_id + n_pts;
	if (use_local_counters)
	{
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
		{
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
						 local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
		for (int row = 0; row < h; ++row)
		{
			for (int col = 0; col < w; ++col)
			{
				if (local_map_pt_mask.at<uchar>(row, col) == 0)
				{
					local_occupied_counter.at<int>(row, col) = 0;
				}
				else
				{
					local_occupied_counter.at<int>(row, col) = local_visit_counter.at<int>(row, col);
				}
			}
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
	}
	else
	{
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
		{
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
						 local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
	}
}

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose)
{
	// printf("updateGridMap\n");
	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	// printf("max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor,
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);

	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	// printf("Received frame %u \n", pts_and_pose->header.seq);

	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
	vector<double> curr_pt_before = {kf_location.x, kf_location.y, kf_location.z};
	vector<double> curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);
	kf_pos_x = curr_pt_after[0] * scale_factor;
	kf_pos_y = curr_pt_after[1] * scale_factor;
	kf_pos_z = curr_pt_after[2] * scale_factor;

	//kf_pos_x = kf_location.x * scale_factor;
	//kf_pos_z = kf_location.z * scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - map_lbound) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - map_bbound) * norm_factor_z));
	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w){
		cout << "kf_pos_grid_x " << kf_pos_grid_x << " kf_pos_x " << kf_pos_x << endl;
		printf("kf_pos_grid_x out of bound!\n");
		return;
	}
	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h){
		printf("kf_pos_grid_z out of bound!");
		return;
	}

	// 新增当前位姿显示
	curpose.header.seq = 0;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id = "map";
	// 地图点需按分辨率映射
	curpose.pose.position.x = kf_pos_grid_x * grid_map_msg.info.resolution;
	curpose.pose.position.y = kf_pos_grid_z * grid_map_msg.info.resolution;
	curpose.pose.position.z = 0;
	// 四元数恢复旋转矩阵
	double ox, oy, oz, ow;
	ox = kf_orientation.x;
	oy = kf_orientation.y;
	oz = kf_orientation.z;
	ow = kf_orientation.w;

		// Eigen::Matrix3d R;
// 	R << 
// 	1-2*oy*oy-2*oz*oz, 	2*ox*oy+2*ow*oz, 	2*ox*oz-2*ow*oy,
// 	2*ox*oy-2*ow*oz, 	1-2*ox*ox-2*oz*oz, 	2*oy*oz+2*ow*ox,
// 	2*ox*oz+2*ow*oy, 	2*oy*oz-2*ow*ox, 	1-2*ox*ox-2*oy*oy;

//   tf::Matrix3x3 tf_camera_rotation (R.at<float> (0,0), R.at<float> (0,1), R.at<float> (0,2),
//                                     R.at<float> (1,0), R.at<float> (1,1), R.at<float> (1,2),
//                                     R.at<float> (2,0), R.at<float> (2,1), R.at<float> (2,2)
//                                    );
//   const tf::Matrix3x3 tf_orb_to_ros (0, 0, 1,
//                                     -1, 0, 0,
//                                      0,-1, 0);

//   //Transform from orb coordinate system to ros coordinate system on camera coordinates
//   tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
// //   tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

//   //Inverse matrix
//   tf_camera_rotation = tf_camera_rotation.transpose();
// //   tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

//   //Transform from orb coordinate system to ros coordinate system on map coordinates
//   tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
//   tf_camera_translation = tf_orb_to_ros*tf_camera_translation;


	Eigen::Matrix3d R;
	R << 
	1-2*oy*oy-2*oz*oz, 	2*ox*oy+2*ow*oz, 	2*ox*oz-2*ow*oy,
	2*ox*oy-2*ow*oz, 	1-2*ox*ox-2*oz*oz, 	2*oy*oz+2*ow*ox,
	2*ox*oz+2*ow*oy, 	2*oy*oz-2*ow*ox, 	1-2*ox*ox-2*oy*oy;
	// // 相机坐标系 to ROS系



	Eigen::Matrix3d T;
	T <<
	0,  0, 1,
	0, -1, 0,
	-1, 0, 0;
	R = T * R;

	// Eigen::Matrix3d T,TR;
	// T <<
	// 0,  0, 1,
	// -1, 0, 0,
	// 0, -1, 0;
	// R = T * R;
	// R = R.inverse();
	// R = T*R;
	// TR <<
	// 1,  0, 0,
	// 0, 0, 1,
	// 0, -1, 0;
	// R=TR*R;

	// kinect rgbd camera
	T << 
	1,  0, 0,
	0,  0, 1,
	0, 1, 0;	
	R = T * R;
		T <<
	1,  0, 0,
	0, 0, -1,
	0, 1, 0;
	R=T*R;

	// T << 
	// 0,  0, -1,
	// 0,  1, 0,
	// 1, 0, 0;	
	// R = T * R;

	// R = R.inverse();
	// R = T*R;
	// 旋转矩阵转四元数
	Eigen::Quaterniond Q( R );
	// 注意以xz面为地图xy平面
	curpose.pose.orientation.x = Q.x();
	curpose.pose.orientation.y = Q.z();
	curpose.pose.orientation.z = Q.y();
	curpose.pose.orientation.w = Q.w();
	Eigen::Vector3d rpy = R.eulerAngles(0,1,2);
	// cout << "Euler Angles of camera pose:\n" << rpy[0]/3.14*180 << " " << rpy[1]/3.14*180 << " " << rpy[2]/3.14*180 << endl;
	// cout << "orientation: " << curpose.pose.orientation.x << " " << curpose.pose.orientation.y << " " << curpose.pose.orientation.z << " " << curpose.pose.orientation.w << endl;
	// pub_pose.publish( curpose );
	
	++n_kf_received;
	unsigned int n_pts = pts_and_pose->poses.size() - 1;
	// printf("Processing key frame %u and %u points\n",n_kf_received, n_pts);
	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

	getGridMap();
}

void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{
	global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int)(all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int)(all_kf_and_pts->poses[0].position.z) != n_kf)
	{
		printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

	kf_plantVec.clear();
	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
	{
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		id += n_pts;
		kf_plantVec.push_back({kf_location.x, kf_location.y, kf_location.z}); //获取所有关键帧点集
	}
	PlaneFittingRansac(kf_plantVec, bestplane); //拟合平面
	vector<double> bestNormalVec = {bestplane[0], bestplane[1], -1};
	auto angle = acos(DotProduct(bestNormalVec, idealNormalVec) / (Normalize(bestNormalVec) * Normalize(idealNormalVec))) * 180 / PI; //Arc TO degree
	if (angle < 20)
	{
		//cout << "Angle between bese&ideal: " << angle << endl;
		rotationMatrix = CalRotationMatrix(bestNormalVec, idealNormalVec); //计算旋转矩阵
	}
	else
	{
		bestNormalVec[0] = -bestNormalVec[0]; //反转法向量
		bestNormalVec[1] = -bestNormalVec[1];
		bestNormalVec[2] = -bestNormalVec[2];

		auto angle = acos(DotProduct(bestNormalVec, idealNormalVec) / (Normalize(bestNormalVec) * Normalize(idealNormalVec))) * 180 / PI; //Arc TO degree

		if (angle < 20)
		{
			//cout << "Angle between bese&ideal: " << angle << endl;
			rotationMatrix = CalRotationMatrix(bestNormalVec, idealNormalVec); //计算旋转矩阵
		}
		else
			cerr << "Angle ERROR!!!!: " << angle << endl;
	}

	for (int i = 0; i < kf_plantVec.size(); i++)
	{
		kf_plantVec[i] = RotationAjust(rotationMatrix, kf_plantVec[i]); //旋转所有关键帧平面位置
	}
	//unsigned int id = 0;
	id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
	{
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts)
		{
		vector<double> curr_pt_before = {kf_location.x, kf_location.y, kf_location.z};
		auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);

		float kf_pos_x = curr_pt_after[0] * scale_factor;
		float kf_pos_z = curr_pt_after[2] * scale_factor;

		//float kf_pos_x = kf_location.x * scale_factor;
		//float kf_pos_z = kf_location.z * scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - map_lbound) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - map_bbound) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size())
		{
			printf("resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				   kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;
	}

	getGridMap();
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
	printf("Done. Time taken: %f secs\n", ttrack);
	pub_grid_map.publish(grid_map_msg);
}
}
void getGridMap()
{
	std::lock_guard<std::mutex> lock(g_gridMapMutex);
	for (int row = 0; row < h; ++row)
	{
		for (int col = 0; col < w; ++col)
		{
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= visit_thresh)
			{
				grid_map.at<float>(row, col) = 0.5;
				//grid_map_proba.at<uchar>(row, col) = 128;
			}
			else
			{
				//grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
				grid_map.at<float>(row, col) = 1.0 - (1.0 * occupieds / visits);

				/*
				if(occupieds > 12)
					grid_map_proba.at<uchar>(row, col) = 0;
				else
					grid_map_proba.at<uchar>(row, col) = (uchar)(255 * (1.0 - 1.0 * occupieds / visits));
				*/
			}
			if (grid_map.at<float>(row, col) >= free_thresh)
			{
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh)
			{
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else
			{
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
			//grid_map_proba.at<uchar>(row, col) = (1 - grid_map.at<float>(row, col)) * 255;
		}
	}

	for (int i = 0; i < kf_plantVec.size(); i++) //清除关键帧路径上的噪声
	{
		int r = 4; //关键帧清除半径 单位：栅格
		float kf_pos_x = kf_plantVec[i][0] * scale_factor;
		float kf_pos_z = kf_plantVec[i][2] * scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - map_lbound) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - map_bbound) * norm_factor_z));

		for (int row = kf_pos_grid_z - r; row <= kf_pos_grid_z + r; row++)
			for (int col = kf_pos_grid_x - r; col <= kf_pos_grid_x + r; col++)
			{
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
	}

	//cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
	//grid_map_thresh_resized = grid_map_thresh.clone();
}

void parseParams(int argc, char **argv)
{
	int arg_id = 1;
	if (argc > arg_id)
	{
		resolution = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		scale_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		resize_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_max_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_min_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_max_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_min_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		free_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		occupied_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		use_local_counters = atoi(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		visit_thresh = atoi(argv[arg_id++]);
	}
}

void printParams()
{
	printf("Using params:\n");
	printf("scale_factor: %f\n", scale_factor);
	printf("resize_factor: %f\n", resize_factor);
	printf("cloud_max: %f, %f\t cloud_min: %f, %f\n", cloud_max_x, cloud_max_z, cloud_min_x, cloud_min_z);
	//printf("cloud_min: %f, %f\n", cloud_min_x, cloud_min_z);
	printf("free_thresh: %f\n", free_thresh);
	printf("occupied_thresh: %f\n", occupied_thresh);
	printf("use_local_counters: %d\n", use_local_counters);
	printf("visit_thresh: %d\n", visit_thresh);
	printf("resolution: %f\n", resolution);
}

/*
平面拟合 方程z=ax+by+c 法向量为[a,b,-1]
vector<double> bestplane(3, 0);// [0]a [1]b [2]c
PlaneFittingRansac(Vectors, bestplane);
cout << "法向量为： x:" << bestplane[0] << " y:" << bestplane[1] << " z:" << -1 << endl;
 */

void PlaneFittingRansac(vector<vector<double>> &Vectors, vector<double> &bestplane)
{
	if (Vectors[0].size() != 3)
		return;

	cout << "data number:" << Vectors.size() << endl;

	int maxinliers = 0;
	srand((unsigned)time(0));
	//迭代30次
	for (int i = 0; i < 30; i++)
	{
		//步骤1：从数据中随机选择3组数据
		//
		set<int> s;
		while (1)
		{
			int r = rand() % (Vectors.size());
			s.insert(r);
			if (s.size() >= 3)
				break;
		}
		int j = 0;
		int select_num[3] = {0};
		for (auto iElement = s.cbegin(); iElement != s.end(); iElement++)
		{
			//cout<<*iElement<<" ";
			select_num[j++] = *iElement;
		}
		//cout << select_num[0] << "," << select_num[1] << "," << select_num[2] << endl;
		//cout << "data1:" << Vectors[select_num[0]][0] << "," << Vectors[select_num[0]][1] << "," << Vectors[select_num[0]][2] << endl;
		//cout << "data2:" << Vectors[select_num[1]][0] << "," << Vectors[select_num[1]][1] << "," << Vectors[select_num[1]][2] << endl;
		//cout << "data3:" << Vectors[select_num[2]][0] << "," << Vectors[select_num[2]][1] << "," << Vectors[select_num[2]][2] << endl;
		//步骤2：通过获得的数据获得模型参数
		double a = ((Vectors[select_num[0]][2] - Vectors[select_num[2]][2]) * (Vectors[select_num[1]][1] - Vectors[select_num[2]][1]) - (Vectors[select_num[1]][2] - Vectors[select_num[2]][2]) * (Vectors[select_num[0]][1] - Vectors[select_num[2]][1])) /
				   ((Vectors[select_num[0]][0] - Vectors[select_num[2]][0]) * (Vectors[select_num[1]][1] - Vectors[select_num[2]][1]) - (Vectors[select_num[1]][0] - Vectors[select_num[2]][0]) * (Vectors[select_num[0]][1] - Vectors[select_num[2]][1]));
		double b = ((Vectors[select_num[1]][2] - Vectors[select_num[2]][2]) - a * (Vectors[select_num[1]][0] - Vectors[select_num[2]][0])) /
				   (Vectors[select_num[1]][1] - Vectors[select_num[2]][1]);
		double c = Vectors[select_num[0]][2] - a * Vectors[select_num[0]][0] - b * Vectors[select_num[0]][1];
		//cout << "a:" << a << ",b:" << b << ",c:" << c << endl;
		//步骤3：统计内点个数
		int inliers = 0;
		int sigma = 1.0; //阈值,可以自己调整
		for (auto k = 0; k < Vectors.size(); k++)
		{

			if (fabs(a * Vectors[k][0] + b * Vectors[k][1] - Vectors[k][2] + c) < sigma)
				inliers++;
		}
		//cout << "inliers" << inliers << endl;
		//步骤4：选出内点个数最大的参数
		if (inliers > maxinliers)
		{
			maxinliers = inliers;
			bestplane.clear();
			bestplane.push_back(a);
			bestplane.push_back(b);
			bestplane.push_back(c);
		}
	}
	//cout << "法向量为： x:" << bestplane[0]/bestplane[1] << " y:" << 1 << " z:" << -1/bestplane[1] << endl;
}

//叉乘
vector<double> CrossProduct(vector<double> &a, vector<double> &b)
{
	if (a.size() != 3 || b.size() != 3)
		exit(-1);

	vector<double> c(3);

	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];

	return c;
}

//点乘
double DotProduct(vector<double> &a, vector<double> &b)
{
	if (a.size() != 3 || b.size() != 3)
		exit(-1);
	return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

//模
double Normalize(vector<double> &v)
{
	if (v.size() != 3)
		exit(-1);

	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

//罗德里格旋转公式(Rodrigues' rotation formula)
vector<vector<double>> RotationMatrix(double angle, vector<double> &u)
{
	double norm = Normalize(u);
	vector<vector<double>> rotatinMatrix(3, vector<double>(3, 0)); //3X3矩阵

	u[0] = u[0] / norm;
	u[1] = u[1] / norm;
	u[2] = u[2] / norm;

	rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
	rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle)) - u[2] * sin(angle);
	rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

	rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
	rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
	rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));

	rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
	rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
	rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

	return rotatinMatrix;
}

//计算旋转矩阵
vector<vector<double>> CalRotationMatrix(vector<double> &vectorBefore, vector<double> &vectorAfter)
{
	vector<double> rotationAxis; //旋转轴
	double rotationAngle;		 //旋转角

	rotationAxis = CrossProduct(vectorBefore, vectorAfter);
	rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
	return RotationMatrix(rotationAngle, rotationAxis);
}

//输入旋转矩阵和向量，输入旋转后向量
inline vector<double> RotationAjust(vector<vector<double>> &r, vector<double> &v)
{
	if (r.size() != 3 || r[0].size() != 3)
		exit(-1);

	vector<double> res(3);

	res[0] = r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2];
	res[1] = r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2];
	res[2] = r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2];

	return res;
}