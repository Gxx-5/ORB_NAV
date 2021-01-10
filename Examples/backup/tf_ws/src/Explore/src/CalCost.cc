#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
// #include "../../../../../../include/System.h"
#include "System.h"
#include "costcube.h"

using namespace std;

// double filter_radius = 0.5;
double shooting_dst = 1.0;
double field_size = 0.25;
double resolution = 0.1;
std::string cloud_name = "/map_cloud";
std::string pose_name = "/camera_pose";
Eigen::Matrix3d Rwc;
Eigen::Vector3d Twc;
cv::Mat costcube_map;

// cv::Mat Tcw,Rcw,Rwc,tcw,twc;//Rotation, translation  and camera center(refer to ORB)
CostCube COSTCUBE;
//Publish
ros::Publisher _pub_map_cloud;
ros::Publisher _pub_cur_view_cloud;
ros::Publisher _vis_pub,_vis_text_pub;
ros::Publisher _costcloud_pub;

void FilterNearbyPoint(ORB_SLAM2::Map* map,std::vector<float> CamPos);
void PublishMapPointstoCloud(std::vector<ORB_SLAM2::MapPoint*> points,ros::Publisher publisher,
								vector<geometry_msgs::Point>* points_pos = NULL);
void VisualizeCostCube(cv::Mat cost_map);
void testColorfunc();
void parseParams(int argc, char **argv);
void printParams();
void preProcess(const sensor_msgs::PointCloud::ConstPtr& pt_cloud,vector<geometry_msgs::Point>& map_points);
void PublishMapPoints(vector<geometry_msgs::Point> map_points,ros::Publisher publisher);
void poseCallback(const geometry_msgs::PoseStamped &pose);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Explore");
	// ros::start();
	ros::NodeHandle n;
	parseParams(argc, argv);
	printParams();
	vector<geometry_msgs::Point> map_points;
	ros::Subscriber pose_sub = n.subscribe(pose_name, 10, &poseCallback);
	ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud>(cloud_name,1,boost::bind(&preProcess,_1,map_points));
	ros::Publisher pt_pub = n.advertise<sensor_msgs::PointCloud>("pt_cloud",10);

	ros::Rate loop_rate(1);
	while(ros::ok()){
		PublishMapPoints(map_points,pt_pub);
		costcube_map = COSTCUBE.calCostCubeByDistance(map_points);
		VisualizeCostCube(costcube_map);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 1;
}

void preProcess(const sensor_msgs::PointCloud::ConstPtr& pt_cloud,vector<geometry_msgs::Point>& map_points){	
	map_points.clear();
	for(int i=0;i<pt_cloud->points.size();++i){
		geometry_msgs::Point point;
		Eigen::Vector3d pos(pt_cloud->points[i].x,pt_cloud->points[i].y,pt_cloud->points[i].z);
		pos = Rwc.inverse()*(pos - Twc);
		point.x = pos(0,0);
		point.y = pos(1,0);
		point.z = pos(2,0);
		map_points.push_back(point);
	}
}

void poseCallback(const geometry_msgs::PoseStamped &pose){
	Eigen::Quaterniond quat;
	quat.x() = pose.pose.orientation.x;
	quat.y() = pose.pose.orientation.y;
	quat.z() = pose.pose.orientation.z;
	quat.w() = pose.pose.orientation.w;

	Rwc = quat.toRotationMatrix();
	Twc(0,0) = pose.pose.position.x;
	Twc(1,0) = pose.pose.position.y;
	Twc(2,0) = pose.pose.position.z;
}

// void init(ros::NodeHandle nodeHandler,double shooting_dst,double field_size,double resolution){
//         COSTCUBE.reinitialize(shooting_dst,field_size,resolution);
//         _focal_len = shooting_dst;
//         _field_size = field_size;
//         resolution = resolution;
// 	_pub_cur_view_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cur_map_cloud", 1000);
// 	_vis_pub = nodeHandler.advertise<visualization_msgs::MarkerArray>("CostCube",10);
// 	_vis_text_pub = nodeHandler.advertise<visualization_msgs::MarkerArray>("CostCubeText",10);
// 	_costcloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud>("costcloud", 50);	
// }

void mainProcess(vector<geometry_msgs::Point> map_points){
       	//Update Rotation, translation  and camera center
	// cv::Mat Tcw = SLAM.getTracker()->mCurrentFrame.mTcw; 
	// cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
	// cv::Mat Rwc = Rcw.t();
	// cv::Mat tcw = Tcw.rowRange(0,3).col(3);
	// cv::Mat twc = -Rwc*tcw;
        //Publish map points
	// vector<geometry_msgs::Point> map_points_pos;
	// std::vector<ORB_SLAM2::MapPoint*> all_map_points = SLAM.getMap()->GetAllMapPoints();	
	// PublishMapPointstoCloud(all_map_points,pub_map_cloud,&map_points_pos);
	// vector<ORB_SLAM2::MapPoint*> cur_view_points = SLAM.GetTrackedMapPoints();
	// map_points_pos = getMapPointsInCameraCoordinates(cur_view_points);
	// PublishMapPointstoCloud(cur_view_points,_pub_cur_view_cloud,&map_points_pos);
	//Publish nearby map points
	// std::vector<ORB_SLAM2::MapPoint*> nearby_points = SLAM.getMap()->GetNearbyMapPoints();
	// vector<geometry_msgs::Point> nearby_points_pos;
	// PublishMapPointstoCloud(nearby_points,pub_cur_map_cloud,&nearby_points_pos);
	costcube_map = COSTCUBE.calCostCubeByDistance(map_points);
	// VisualizeCostCube(COSTCUBE.dst_mat,camera_pose);
	VisualizeCostCube(costcube_map);
	// testColorfunc();
}

geometry_msgs::Pose getCamPose(cv::Mat twc,cv::Mat Rwc){
	geometry_msgs::Pose camera_pose;
	vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
	camera_pose.position.x = twc.at<float>(0);
	camera_pose.position.y = twc.at<float>(1);
	camera_pose.position.z = twc.at<float>(2);
	camera_pose.orientation.x = q[0];
	camera_pose.orientation.y = q[1];
	camera_pose.orientation.z = q[2];
	camera_pose.orientation.w = q[3];
	return camera_pose;
}

// tf::Transform getTFfromPose(geometry_msgs::Pose campose){
// 	tf::Transform map2cam = 
// 		tf::Transform(tf::Quaternion(campose.orientation.x, campose.orientation.y, campose.orientation.z, campose.orientation.w),
// 						tf::Vector3(campose.position.x, campose.position.y,campose.position.z));
// 	return map2cam;
// }

// std::vector<geometry_msgs::Point> getMapPointsInCameraCoordinates(std::vector<ORB_SLAM2::MapPoint*> map_points){
// 	vector<geometry_msgs::Point> cam_points;
// 	int n_map_pts = map_points.size();
// 	for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
// 		if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
// 			//printf("Point %d is bad\n", pt_id);
// 			continue;
// 		}
// 		// 3D in absolute coordinates
// 		cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();
// 		if (wp.empty()) {
// 			//printf("World position for point %d is empty\n", pt_id);
// 			continue;
// 		}
// 		// 3D in camera coordinates
// 		const cv::Mat Pc = Rcw*wp+tcw;
// 		geometry_msgs::Point pt;
// 		pt.x = Pc.at<float>(0);
// 		pt.y = Pc.at<float>(1);
// 		pt.z = Pc.at<float>(2);
// 		// Check positive depth
// 		if(pt.z<0.0f)
// 			continue;
// 		cam_points.push_back(pt);
// 	return cam_points;
// 	}
// }

// void FilterNearbyPoint(ORB_SLAM2::Map* map,vector<float> CamPos){
// 	std::vector<ORB_SLAM2::MapPoint*> nearby_points = map->GetNearbyMapPoints();
// 	int num_pts = nearby_points.size();
// 	int num_filtered = 0;
// 	for (int pt_id = 1; pt_id <= num_pts; ++pt_id){
// 		if (!nearby_points[pt_id - 1] || nearby_points[pt_id - 1]->isBad()) {
// 			// printf("Point %d is bad\n", pt_id);
// 			continue;
// 		}
// 		cv::Mat wp = nearby_points[pt_id - 1]->GetWorldPos();
// 		if (wp.empty()) {
// 			// printf("World position for point %d is empty\n", pt_id);
// 			continue;
// 		}
// 		double dst = sqrt(pow((wp.at<float>(0)-CamPos[0]),2)+pow((wp.at<float>(1)-CamPos[1]),2)+pow((wp.at<float>(2)-CamPos[2]),2));
// 		// cout << "Distance from current point  is " << dst << endl;
// 		if(dst > _filter_radius){
// 			map->EraseNearbyMapPoint(nearby_points[pt_id]);
// 			num_filtered++;
// 		}
// 	}
// 	cout << "total points num " << num_pts << " filtered  " << num_filtered << " points " << endl;
// }

// void PublishMapPointstoCloud(std::vector<ORB_SLAM2::MapPoint*> points,ros::Publisher publisher,
// 								vector<geometry_msgs::Point>* points_pos){
// 	int all_map_pts = points.size();
// 	// Publish all map points 
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);	
// 	int pts_num = 0;
// 	for (int pt_id = 1; pt_id <= all_map_pts; ++pt_id){
// 		if (!points[pt_id - 1] || points[pt_id - 1]->isBad()) {
// 			// printf("Point %d is bad\n", pt_id);
// 			continue;
// 		}
// 		cv::Mat wp = points[pt_id - 1]->GetWorldPos();
// 		if (wp.empty()) {
// 			// printf("World position for point %d is empty\n", pt_id);
// 			continue;
// 		}
// 		if(points_pos){
// 			geometry_msgs::Point pt_pos;
// 			pt_pos.x = wp.at<float>(0);
// 			pt_pos.y = wp.at<float>(1);
// 			pt_pos.z = wp.at<float>(2);
// 			points_pos->push_back(pt_pos);
// 			pts_num++;
// 		}
// 		map_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
// 		//printf("Done getting map point %d\n", pt_id);
// 	}
// 	// if(points_pos){
// 	// 	cout << "pts_num in pointscloud is " << pts_num << endl;
// 	// }
// 	sensor_msgs::PointCloud2 ros_map_cloud;
// 	pcl::toROSMsg(*map_cloud, ros_map_cloud);
// 	ros_map_cloud.header.frame_id = "map";
// 	publisher.publish(ros_map_cloud);
// }

void PublishMapPoints(vector<geometry_msgs::Point> map_points,ros::Publisher publisher){
	int size = map_points.size();
	sensor_msgs::PointCloud pt_cloud;
	pt_cloud.header.stamp = ros::Time::now();
	pt_cloud.header.frame_id = "camera_link";	
	pt_cloud.points.resize(size);

	for(int i=0;i<size;++i){
		pt_cloud.points[i].x = map_points[i].x;
		pt_cloud.points[i].y = map_points[i].y;
		pt_cloud.points[i].z = map_points[i].z;
	}
	publisher.publish(pt_cloud);
}

vector<int> getColor(int value){
	vector<int> startColor{0,255,0};
	vector<int> endColor{255,255,0};	
	if(value >= 255)
		return endColor;
	else if(value <= 0)
		return startColor;

	int r_gap=endColor[0]-startColor[0];
	int g_gap=endColor[1]-startColor[1];
	int b_gap=endColor[2]-startColor[2];
	
	// int nSteps = max(abs(r), max(abs(g), abs(b)));
	// if (nSteps < 1) nSteps = 1;
	int nSteps = 255;
	// Calculate the step size for each color
	float rStep=r_gap/(float)nSteps;
	float gStep=g_gap/(float)nSteps;
	float bStep=b_gap/(float)nSteps;

	// Reset the colors to the starting position
	float rStart=startColor[0];
	float gStart=startColor[1];
	float bStart=startColor[2];	

	// float step = (value - 255)/255;
	int step = value;
	int r = rStart + r_gap * value / 255;
	int g = gStart + g_gap * value / 255;
	int b = bStart + b_gap * value / 255;
	float a = value / 255;
	// cout << r << " " << g << " " << b << endl;
	return vector<int>{r,g,b};
	// return vector<int>{(int)(rStart+rStep*step+0.5),(int)(gStart+gStep*step+0.5),(int)(bStart+bStep*step+0.5)};
}
void testColorfunc(ros::Publisher vis_pub,ros::Publisher vis_text_pub){
	float resolution = 10.0/255;
	visualization_msgs::MarkerArray markerArr;
	visualization_msgs::MarkerArray markerTextArr;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_text;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "";
	marker.lifetime = ros::Duration();	
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = resolution;
	marker.scale.y = resolution;
	marker.scale.z = resolution;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	int marker_id = 0;

	marker_text.header.frame_id = "map";
	marker_text.header.stamp = ros::Time::now();
	marker_text.ns = "";
	marker_text.lifetime = ros::Duration();	
	marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_text.action = visualization_msgs::Marker::MODIFY;
	marker_text.pose.orientation.x = 0.0;
	marker_text.pose.orientation.y = 0.0;
	marker_text.pose.orientation.z = 0.0;
	marker_text.pose.orientation.w = 1.0;
	marker_text.scale.z = resolution;
	marker_text.color.a = 1.0; // Don't forget to set the alpha!!

	for(int i =0; i<255;++i){
		marker.pose.position.x = -5.0+i*resolution;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		vector<int> color  = getColor(i);
		marker.color.r =  color[0];
		marker.color.g = color[1];
		marker.color.b = color[2];
		marker.id = marker_id++;
		markerArr.markers.push_back(marker);
		marker_text.pose = marker.pose;
		string str = to_string(i) + ", " + to_string(color[0]) + ", " +  to_string(color[1]) + ", " +  to_string(color[2]);
		marker_text.text = str;//to_string(i);
		marker_text.id = marker_id - 1;
		markerTextArr.markers.push_back(marker_text);
	}
	vis_pub.publish(markerArr);
	vis_text_pub.publish(markerTextArr);
	cout << "test done." << endl;
}

void VisualizeCostCube(cv::Mat cost_map){
	if(cost_map.empty()){
		cout << "CostCube map is empty." << endl;
		return;
	}
	visualization_msgs::MarkerArray markerArr;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "camera_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "";
	marker.lifetime = ros::Duration();	
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::MODIFY;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.8 * resolution;
	marker.scale.y = 0.8 * resolution;
	marker.scale.z = 0.8 * resolution;
	marker.color.a = 0.1; // Don't forget to set the alpha!

	visualization_msgs::MarkerArray markerTextArr;
	visualization_msgs::Marker marker_text;
	marker_text.header.frame_id = "camera_link";
	marker_text.header.stamp = ros::Time::now();
	marker_text.ns = "";
	marker_text.lifetime = ros::Duration();	
	marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker_text.action = visualization_msgs::Marker::MODIFY;
	marker_text.pose.orientation.x = 0.0;
	marker_text.pose.orientation.y = 0.0;
	marker_text.pose.orientation.z = 0.0;
	marker_text.pose.orientation.w = 1.0;
	marker_text.scale.z = 0.3 * resolution;
	marker_text.color.a = 1.0; // Don't forget to set the alpha!!

	vector<int> cam_posid{int(field_size / resolution),int(field_size / resolution),0};
	// cout << int(field_size / resolution) << " " << cost_map.size[0] << endl;
	int marker_id = 0;
	for (int row = 0; row < cost_map.size[0]; ++row){
		for (int col = 0; col < cost_map.size[1]; ++col){
                        for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				float cur_cost = cost_map.at<float>(row, col, hei);
				// cur_cost = int(cur_cost*100)/100.0;
				// cout << cur_cost;
				// cout << "cur_cost:" << cur_cost << " ";
				vector<int> color  = getColor(cur_cost);
				// marker.pose.position.x = camera_pose.position.x + (row - cam_posid[0]) * resolution;
				// marker.pose.position.y = camera_pose.position.y + (col - cam_posid[1]) * resolution;
				// marker.pose.position.z = camera_pose.position.z + (hei - cam_posid[2]) * resolution;
				marker.pose.position.x =  (row - cam_posid[0]) * resolution;
				marker.pose.position.y =  (col - cam_posid[1]) * resolution;
				marker.pose.position.z =  (hei - cam_posid[2]) * resolution;
				marker.color.r =  color[0];
				marker.color.g = color[1];
				marker.color.b = color[2];
				marker.id = marker_id++;
				markerArr.markers.push_back(marker);
				marker_text.pose = marker.pose;
				marker_text.text = to_string(cur_cost).substr(0,4);
				// cout << to_string(cur_cost).substr(0,4);
				marker_text.id = marker_id - 1;
				markerTextArr.markers.push_back(marker_text);
			}
			// cout << endl;
		}
		// cout << endl;
	}
	// cout << endl << endl;
	_vis_pub.publish(markerArr);
	_vis_text_pub.publish(markerTextArr);

	sensor_msgs::PointCloud cloud;
	int num_points = cost_map.size[0]*cost_map.size[1]*cost_map.size[2];
	cloud.header.stamp = ros::Time::now();
    	cloud.header.frame_id = "map";//填充 PointCloud 消息的头：frame 和 timestamp．
    	cloud.points.resize(num_points);//设置点云的数量．
 
    	//增加信道 "intensity" 并设置其大小，使与点云数量相匹配．
    	cloud.channels.resize(1);
    	cloud.channels[0].name = "intensities";
    	cloud.channels[0].values.resize(num_points);

	//使用虚拟数据填充 PointCloud 消息．同时，使用虚拟数据填充 intensity 信道．
	int i = 0;
    	for (int row = 0; row < cost_map.size[0]; ++row)
		for (int col = 0; col < cost_map.size[1]; ++col)
                        for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				cloud.points[i].x = (row - cam_posid[0]) * resolution;
				cloud.points[i].y = (col - cam_posid[1]) * resolution;
				cloud.points[i].z =  (hei - cam_posid[2]) * resolution;
				float cur_cost = cost_map.at<float>(row, col, hei);
				cloud.channels[0].values[i] = int(cur_cost*100);
				i++;
    			}
    _costcloud_pub.publish(cloud);
}

void parseParams(int argc, char **argv)
{	
	int arg_id = 1;
	if (argc > arg_id)
	{
		shooting_dst = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		field_size = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		resolution = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_name = argv[arg_id++];
	}
	if (argc > arg_id)
	{
		pose_name = argv[arg_id++];
	}
}

void printParams()
{
	printf("Using params:\n");
	printf("shooting_dst: %f\n", shooting_dst);
	printf("field_size: %f\n", field_size);
	printf("resolution: %f\n", resolution);
	printf("cloud_topic_name: %s\n", cloud_name);
	printf("pose_topic_name: %s\n", pose_name);
}