/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <time.h>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include<opencv2/core/core.hpp>
#include "../../../../include/System.h"

#include "MapPoint.h"
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>
#include "Explore/include/costcube.h"

//! parameters
bool read_from_topic = true, read_from_camera = false;
// double nearbyradius = 1.0;
//Publish
ros::Publisher pub_cloud;
ros::Publisher pub_map_cloud;
ros::Publisher pub_cur_view_cloud;
ros::Publisher vis_pub,vis_text_pub;
bool save_to_results = false;
std::string image_topic = "/camera/image_raw";
int all_pts_pub_gap = 0;

vector<string> vstrImageFilenames;
vector<double> vTimestamps;
cv::VideoCapture cap_obj;

bool pub_all_pts = false;
int pub_count = 0;
double filter_radius = 0.5;
double focal_len = 1.5;
double field_size = 0.5;
double costcube_resolution = 0.1;
cv::Mat costcube_map;
CostCube COSTCUBE(focal_len,field_size,costcube_resolution);

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
	vector<double> &vTimestamps);
inline bool isInteger(const std::string & s);
void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cur_camera_pose, int frame_id);
void FilterNearbyPoint(ORB_SLAM2::Map* map,std::vector<float> CamPos);
void PublishMapPointstoCloud(std::vector<ORB_SLAM2::MapPoint*> points,ros::Publisher publisher,
								vector<geometry_msgs::Point>* points_pos = NULL);
void VisualizeCostCube(cv::Mat cost_map,geometry_msgs::Pose camera_pose);

class ImageGrabber{
public:
	ImageGrabber(ORB_SLAM2::System &_SLAM, ros::Publisher &_pub_pts_and_pose,
		ros::Publisher &_pub_all_kf_and_pts, ros::Publisher &_pub_cur_camera_pose) :
		SLAM(_SLAM), pub_pts_and_pose(_pub_pts_and_pose),
		pub_all_kf_and_pts(_pub_all_kf_and_pts), pub_cur_camera_pose(_pub_cur_camera_pose), frame_id(0){}

	void GrabImage(const sensor_msgs::ImageConstPtr& msg);

	ORB_SLAM2::System &SLAM;
	ros::Publisher &pub_pts_and_pose;
	ros::Publisher &pub_all_kf_and_pts;
	ros::Publisher &pub_cur_camera_pose;
	int frame_id;
};
bool parseParams(int argc, char **argv);

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "Monopub");
	ros::start();
	if (!parseParams(argc, argv)) {
		return EXIT_FAILURE;
	}
	int n_images = vstrImageFilenames.size();

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
	ros::NodeHandle nodeHandler;
	pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	pub_map_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("map_cloud", 1000);
	pub_cur_view_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cur_map_cloud", 1000);
	vis_pub = nodeHandler.advertise<visualization_msgs::MarkerArray>("CostCube",10);
	vis_text_pub = nodeHandler.advertise<visualization_msgs::MarkerArray>("CostCubeText",10);
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	// ros::Publisher pub_cur_camera_pose = nodeHandler.advertise<geometry_msgs::Pose>("/cur_camera_pose", 1000);
	 ros::Publisher pub_cur_camera_pose = nodeHandler.advertise<geometry_msgs::PoseStamped>("/cur_camera_pose", 1000);
	//Param 

	if (read_from_topic) {
		ImageGrabber igb(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cur_camera_pose);
		ros::Subscriber sub = nodeHandler.subscribe(image_topic, 1, &ImageGrabber::GrabImage, &igb);
		ros::spin();
	}
	else{
		ros::Rate loop_rate(5);
		cv::Mat im;
		double tframe = 0;
#ifdef COMPILEDWITHC11
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
		std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
		for (int frame_id = 0; read_from_camera || frame_id < n_images; ++frame_id){
			if (read_from_camera) {
				cap_obj.read(im);
#ifdef COMPILEDWITHC11
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
				std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
				tframe = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
				//printf("fps: %f\n", 1.0 / tframe);
			}
			else {
				// Read image from file
				im = cv::imread(vstrImageFilenames[frame_id], CV_LOAD_IMAGE_UNCHANGED);
				tframe = vTimestamps[frame_id];
			}
			if (im.empty()){
				cerr << endl << "Failed to load image at: " << vstrImageFilenames[frame_id] << endl;
				return 1;
			}
			// Pass the image to the SLAM system
			cv::Mat curr_pose = SLAM.TrackMonocular(im, tframe);

			publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cur_camera_pose, frame_id);

			//cv::imshow("Press escape to exit", im);
			//if (cv::waitKey(1) == 27) {
			//	break;
			//}
			ros::spinOnce();
			loop_rate.sleep();
			if (!ros::ok()){ break; }
		}
	}
	//ros::spin();
	if(save_to_results){
		mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
		SLAM.getMap()->Save("results//map_pts_out.obj");
		SLAM.getMap()->SaveWithTimestamps("results//map_pts_and_keyframes.txt");
		// Save camera trajectory
		SLAM.SaveKeyFrameTrajectoryTUM("results//key_frame_trajectory.txt");

		cout << "Press 'q' in the Frame Window to quit!" << endl;
		while (cv::waitKey(0) != 'q') { }
	}
	// Stop all threads
	SLAM.Shutdown();
	//geometry_msgs::PoseArray pt_array;
	//pt_array.header.seq = 0;
	//pub_pts_and_pose.publish(pt_array);
	ros::shutdown();
	return 0;
}

void publish(ORB_SLAM2::System &SLAM, ros::Publisher &pub_pts_and_pose,
			 ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cur_camera_pose, int frame_id) {
	if (all_pts_pub_gap>0 && pub_count >= all_pts_pub_gap) {
		pub_all_pts = true;
		pub_count = 0;
	}

	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {
		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM2::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM2::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (key_frame->isBad())
				continue;

			cv::Mat R = key_frame->GetRotation().t();
			vector<float> q = ORB_SLAM2::Converter::toQuaternion(R);
			cv::Mat twc = key_frame->GetCameraCenter();
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			unsigned int n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM2::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					// printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = map_pt->GetWorldPos();
				if (pt_pose.empty()) {
					// printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				// pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			geometry_msgs::Pose n_pts_msg;
			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
			kf_pt_array.poses[n_pts_id] = n_pts_msg;
			++n_kf;
		}
		geometry_msgs::Pose n_kf_msg;
		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
		kf_pt_array.poses[0] = n_kf_msg;
		kf_pt_array.header.frame_id = "map";
		// kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyfranmes\n", n_kf);
		pub_all_kf_and_pts.publish(kf_pt_array);
		SLAM.getMap()->ResetNearbyPoint();//when loop closure detected,reset nearbypoints to map points(all)
	}
	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
		++pub_count;
		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
		ORB_SLAM2::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
		//while (pKF->isBad())
		//{
		//	Trw = Trw*pKF->mTcp;
		//	pKF = pKF->GetParent();
		//}

		vector<ORB_SLAM2::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM2::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		Trw = Trw*pKF->GetPose()*Two;
		cv::Mat lit = SLAM.getTracker()->mlRelativeFramePoses.back();
		cv::Mat Tcw = lit*Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
		//geometry_msgs::Pose camera_pose;
		// std::vector<ORB_SLAM2::MapPoint*> all_map_points = SLAM.getMap()->GetAllMapPoints();
		std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();
		// int all_map_pts = all_map_points.size();

		// printf("\ntracked_map_pts: %d\n", n_map_pts);
		// printf("all_map_pts: %d\n", all_map_pts);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		//printf("Done getting camera pose\n");

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){

			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				//printf("Point %d is bad\n", pt_id);
				continue;
			}
			cv::Mat wp = map_points[pt_id - 1]->GetWorldPos();

			if (wp.empty()) {
				//printf("World position for point %d is empty\n", pt_id);
				continue;
			}
			geometry_msgs::Pose curr_pt;
			//printf("wp size: %d, %d\n", wp.rows, wp.cols);
			pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		ros_cloud.header.frame_id = "map";
		// ros_cloud.header.seq = ni;

		// printf("valid map pts: %lu\n", pt_array.poses.size()-1);				
		// printf("ros_cloud size: %d x %d\n", ros_cloud.height, ros_cloud.width);
		pub_cloud.publish(ros_cloud);

		pt_array.header.frame_id = "map";
		pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose.publish(pt_array);
		// pub_kf.publish(camera_pose);
	}
	// Publish current camera pose
	geometry_msgs::Pose camera_pose;
	if (!SLAM.getTracker()->mCurrentFrame.mTcw.empty())
	{
		cv::Mat Tcw = SLAM.getTracker()->mCurrentFrame.mTcw; 
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
		
		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];
		
		geometry_msgs::PoseStamped camera_poseStamped;
		camera_poseStamped.pose = camera_pose;
		camera_poseStamped.header.frame_id = "map";
		camera_poseStamped.header.stamp = ros::Time::now();
		pub_cur_camera_pose.publish(camera_poseStamped);
		//Modify Map points nearby camear by the way
		// FilterNearbyPoint(SLAM.getMap(),std::vector<float>{twc.at<float>(0),twc.at<float>(1),twc.at<float>(2)});
	}
	//Publish map points
	vector<geometry_msgs::Point> map_points_pos;
	// std::vector<ORB_SLAM2::MapPoint*> all_map_points = SLAM.getMap()->GetAllMapPoints();	
	// PublishMapPointstoCloud(all_map_points,pub_map_cloud,&map_points_pos);
	std::vector<ORB_SLAM2::MapPoint*> cur_view_points = SLAM.GetTrackedMapPoints();
	PublishMapPointstoCloud(cur_view_points,pub_cur_view_cloud,&map_points_pos);
	//Publish nearby map points
	// std::vector<ORB_SLAM2::MapPoint*> nearby_points = SLAM.getMap()->GetNearbyMapPoints();
	// vector<geometry_msgs::Point> nearby_points_pos;
	// PublishMapPointstoCloud(nearby_points,pub_cur_map_cloud,&nearby_points_pos);
	costcube_map = COSTCUBE.calCostCubeByDistance(map_points_pos,camera_pose);
	VisualizeCostCube(costcube_map,camera_pose);
}

void FilterNearbyPoint(ORB_SLAM2::Map* map,std::vector<float> CamPos){
	std::vector<ORB_SLAM2::MapPoint*> nearby_points = map->GetNearbyMapPoints();
	int num_pts = nearby_points.size();
	int num_filtered = 0;
	for (int pt_id = 1; pt_id <= num_pts; ++pt_id){
		if (!nearby_points[pt_id - 1] || nearby_points[pt_id - 1]->isBad()) {
			// printf("Point %d is bad\n", pt_id);
			continue;
		}
		cv::Mat wp = nearby_points[pt_id - 1]->GetWorldPos();

		if (wp.empty()) {
			// printf("World position for point %d is empty\n", pt_id);
			continue;
		}
		double dst = sqrt(pow((wp.at<float>(0)-CamPos[0]),2)+pow((wp.at<float>(1)-CamPos[1]),2)+pow((wp.at<float>(2)-CamPos[2]),2));
		// cout << "Distance from current point  is " << dst << endl;
		if(dst > filter_radius){
			map->EraseNearbyMapPoint(nearby_points[pt_id]);
			num_filtered++;
		}
	}
	cout << "total points num " << num_pts << " filtered  " << num_filtered << " points " << endl;
}

void PublishMapPointstoCloud(std::vector<ORB_SLAM2::MapPoint*> points,ros::Publisher publisher,
								vector<geometry_msgs::Point>* points_pos){
	int all_map_pts = points.size();
	// Publish all map points 
	pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);	
	int pts_num = 0;
	for (int pt_id = 1; pt_id <= all_map_pts; ++pt_id){
		if (!points[pt_id - 1] || points[pt_id - 1]->isBad()) {
			// printf("Point %d is bad\n", pt_id);
			continue;
		}
		cv::Mat wp = points[pt_id - 1]->GetWorldPos();

		if (wp.empty()) {
			printf("World position for point %d is empty\n", pt_id);
			continue;
		}
		if(points_pos){
			geometry_msgs::Point pt_pos;
			pt_pos.x = wp.at<float>(0);
			pt_pos.y = wp.at<float>(1);
			pt_pos.z = wp.at<float>(2);
			points_pos->push_back(pt_pos);
			pts_num++;
		}		
		map_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
		//printf("Done getting map point %d\n", pt_id);
	}
	// if(points_pos){
	// 	cout << "pts_num in pointscloud is " << pts_num << endl;
	// }
	sensor_msgs::PointCloud2 ros_map_cloud;
	pcl::toROSMsg(*map_cloud, ros_map_cloud);
	ros_map_cloud.header.frame_id = "map";
	publisher.publish(ros_map_cloud);
}

vector<int> getColor(int value){
	vector<int> startColor{0,255,0};
	vector<int> endColor{255,0,0};	
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
	float fr=startColor[0];
	float fg=startColor[1];
	float fb=startColor[2];	

	// float step = (value - 255)/255;
	int step = value;
	return vector<int>{(int)(fr+rStep*step+0.5),(int)(fg+gStep*step+0.5),(int)(fb+bStep*step+0.5)};
}

void VisualizeCostCube(cv::Mat cost_map,geometry_msgs::Pose camera_pose){
	if(cost_map.empty()){
		cout << "CostCube map is empty." << endl;
		return;
	}
	visualization_msgs::MarkerArray markerArr;
	visualization_msgs::Marker marker;
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
	marker.scale.x = 0.8 * costcube_resolution;
	marker.scale.y = 0.8 * costcube_resolution;
	marker.scale.z = 0.8 * costcube_resolution;
	marker.color.a = 0.1; // Don't forget to set the alpha!

	visualization_msgs::MarkerArray markerTextArr;
	visualization_msgs::Marker marker_text;
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
	marker_text.scale.z = 0.3 * costcube_resolution;
	marker_text.color.a = 1.0; // Don't forget to set the alpha!

	vector<int> cam_posid{int(field_size / costcube_resolution),int(field_size / costcube_resolution),0};
	int marker_id = 0;
	for (int row = 0; row < cost_map.size[0]; ++row){
		for (int col = 0; col < cost_map.size[1]; ++col){
                        for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				int cur_cost =  cost_map.at<float>(row, col, hei);
				// cout << "cur_cost:" << cur_cost << " ";
				vector<int> color  = getColor(cur_cost);
				marker.pose.position.x = camera_pose.position.x + (row - cam_posid[0]) * costcube_resolution;
				marker.pose.position.y = camera_pose.position.y + (col - cam_posid[1]) * costcube_resolution;
				marker.pose.position.z = camera_pose.position.z + (hei - cam_posid[2]) * costcube_resolution;
				marker.color.r =  color[0];
				marker.color.g = color[1];
				marker.color.b = color[2];
				marker.id = marker_id++;
				markerArr.markers.push_back(marker);
				marker_text.pose = marker.pose;
				marker_text.text = to_string(cur_cost);
				marker_text.id = marker_id - 1;
				markerTextArr.markers.push_back(marker_text);
			}
			// cout << endl;
		}
		// cout << endl;
	}
	// cout << endl << endl;
	vis_pub.publish(markerArr);
	vis_text_pub.publish(markerTextArr);
}

inline bool isInteger(const std::string & s){
	if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

	char * p;
	strtol(s.c_str(), &p, 10);

	return (*p == 0);
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps){
	ifstream fTimes;
	string strPathTimeFile = strPathToSequence + "/timestamps.txt";
	fTimes.open(strPathTimeFile.c_str());
	while (!fTimes.eof()){
		string s;
		getline(fTimes, s);
		if (!s.empty()){
			stringstream ss;
			ss << s;
			double t;
			ss >> t;
			vTimestamps.push_back(t);
		}
	}

	string strPrefixLeft = strPathToSequence + "/data/";

	const int nTimes = vTimestamps.size();
	vstrImageFilenames.resize(nTimes);

	for (int i = 0; i < nTimes; i++)
	{
		stringstream ss;
		ss << setfill('0') << setw(10) << i;
		vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
	}
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg){
	// Copy the ros image message to cv::Mat.
	cv_bridge::CvImageConstPtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvShare(msg);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	SLAM.TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
	publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cur_camera_pose, frame_id);
	++frame_id;
}

bool parseParams(int argc, char **argv) {
	if (argc < 4){
		cerr << endl << "Usage: rosrun ORB_SLAM2 Monopub path_to_vocabulary path_to_settings path_to_sequence/camera_id/-1 <image_topic>" << endl;
		return 1;
	}
	if (isInteger(std::string(argv[3]))) {
		int camera_id = atoi(argv[3]);
		if (camera_id >= 0){
			read_from_camera = true;
			printf("Reading images from camera with id %d\n", camera_id);
			cap_obj.open(camera_id);
			if (!(cap_obj.isOpened())) {
				printf("Camera stream could not be initialized successfully\n");
				ros::shutdown();
				return 0;
			}
			int img_height = cap_obj.get(CV_CAP_PROP_FRAME_HEIGHT);
			int img_width = cap_obj.get(CV_CAP_PROP_FRAME_WIDTH);
			printf("Images are of size: %d x %d\n", img_width, img_height);
		}
		else {
			read_from_topic = true;
			if (argc > 4){
				image_topic = std::string(argv[4]);
			}
			printf("Reading images from topic %s\n", image_topic.c_str());
		}
	}
	else {
		LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
	}
	if (argc >= 5) {
		all_pts_pub_gap = atoi(argv[4]);
	}
	printf("all_pts_pub_gap: %d\n", all_pts_pub_gap);
	return 1;
}




