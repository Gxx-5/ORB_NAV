#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "tf/transform_datatypes.h"//转换函数头文件

using namespace ros;



// tf::StampedTransform map2odom_;
tf::Transform map2odom_;


void gridPoseCallback(const geometry_msgs::PoseStamped grid_pose)
{

	// tf::Quaternion quat = tf::Quaternion(grid_pose.pose.orientation.x, grid_pose.pose.orientation.y, grid_pose.pose.orientation.z, grid_pose.pose.orientation.w);
	
    // tf::Matrix3x3 R = tf::Matrix3x3(quat);//进行转换
	// tf::Matrix3x3	T(
	//  1,0,0,
	//  0,0,-1,
	//  0,1,0);

	// R = T * R;

	// double roll, pitch, yaw;//定义存储r\p\y的容器
	// R.getRPY(roll, pitch, yaw);//进行转换
	// tf::Quaternion map2base_quat = tf::Quaternion( roll,  pitch,  yaw);//返回四元数	


	ros::Duration transform_tolerance_ (0.0);
	tf::TransformListener listener;
	try
    {  tf::Quaternion map2base_quat = tf::Quaternion(grid_pose.pose.orientation.x, grid_pose.pose.orientation.y, grid_pose.pose.orientation.z, grid_pose.pose.orientation.w);
		ros::Time now = ros::Time(0);
		tf::Transform map2base = 
		tf::Transform(map2base_quat,//tf::Quaternion(grid_pose.pose.orientation.x, grid_pose.pose.orientation.y, grid_pose.pose.orientation.z, grid_pose.pose.orientation.w),
		   	 							tf::Vector3(grid_pose.pose.position.x, grid_pose.pose.position.y,grid_pose.pose.position.z));

		tf::StampedTransform odom2base;
		// printf("base2map,%f,%f,%f\n%f,%f,%f\n",base2map.getRotation()[0],base2map.getRotation()[1],base2map.getRotation()[2],base2map.getOrigin()[0],base2map.getOrigin()[1],base2map.getOrigin()[2]);									  
		listener.waitForTransform("odom","base_link",now,ros::Duration(2));
		listener.lookupTransform("odom", "base_link",now, odom2base);
		printf("odom2base,%f,%f,%f\n%f,%f,%f\n",odom2base.getRotation()[0],odom2base.getRotation()[1],odom2base.getRotation()[2],odom2base.getOrigin()[0],odom2base.getOrigin()[1],odom2base.getOrigin()[2]);									  

		tf::Transform base2odom = odom2base.inverse();
		tf::Transform base2map = map2base.inverse();
		printf("base2odom,%f,%f,%f\n%f,%f,%f\n",base2odom.getRotation()[0],base2odom.getRotation()[1],base2odom.getRotation()[2],base2odom.getOrigin()[0],base2odom.getOrigin()[1],base2odom.getOrigin()[2]);									  
		ROS_INFO("%lf",now.toSec());
		tf::Transform latest_tf;
		latest_tf = base2map.inverse()*base2odom;

	ros::Time transform_expiration = (ros::Time::now() +transform_tolerance_);
	// tf::StampedTransform map2odom(latest_tf,
	// 									transform_expiration,
	// 									"map", "odom");
    map2odom_ = latest_tf;
	printf("[%lf] send transform from map to odom!\n",transform_expiration.toSec());
	}
	catch (tf::TransformException &ex) {
	ROS_ERROR("%s",ex.what());
	// ros::Duration(1.0).sleep();
	}
}


int main(int argc, char** argv){
    ros::init(argc, argv, "map2odom_Node");    
    ros::NodeHandle n;

	map2odom_ = tf::Transform(tf::Quaternion(0,0,0,1),
		   	 							tf::Vector3(0,0,0));

	tf::TransformBroadcaster broadcaster;
    ros::Subscriber gridpose_sub = n.subscribe("/grid_pose", 5, gridPoseCallback);
    //发送消息的频率为2Hz， 1秒发2个，周期为500ms
    ros::Rate loop_rate(100);
    while(ok()){
        broadcaster.sendTransform(tf::StampedTransform(map2odom_,ros::Time::now(),"map", "odom"));
		printf("map2odom_\n%f,%f,%f\n%f,%f,%f\n",map2odom_.getRotation()[0],map2odom_.getRotation()[1],map2odom_.getRotation()[2],map2odom_.getOrigin()[0],map2odom_.getOrigin()[1],map2odom_.getOrigin()[2]);									  
         //靠sleep()函数保证连续两个消息之间的时间恰好为一个周期
	    ros::spinOnce();
		loop_rate.sleep();		
    }
    return 0;
}