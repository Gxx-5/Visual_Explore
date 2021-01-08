#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "tf/transform_datatypes.h"//转换函数头文件

static geometry_msgs::Pose _pose;

void PoseStampedCallback(geometry_msgs::PoseStamped pose){
	_pose = pose.pose;
}

void PoseCallback(geometry_msgs::Pose pose){
	_pose = pose;
}

void initpose(geometry_msgs::Pose pose){
	pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
	pose.orientation.w = 1;
	pose.position.x = pose.position.y = pose.position.z = 0; 
}

int main(int argc, char** argv){
	ros::init(argc, argv, "EstablishTF_node");    
	ros::NodeHandle n;
	if(argc < 4){
		std::cout << "Too few input parameters , need at least 3 parameters.(parent_frame,child_frame,pose_name)"<< std::endl;
		return 0;
	}
	std::string parent_frame = argv[1];
	std::string child_frame = argv[2];
	std::string pose_name = argv[3];
	int rate = 10;
	if(argc > 4)
		rate = atoi(argv[4]);
	initpose(_pose);

	ros::Subscriber pose_sub = n.subscribe(pose_name, 5, PoseStampedCallback);
	tf::TransformBroadcaster broadcaster;
	
	ros::Rate loop_rate(rate);
	while(ros::ok()){
		tf::Transform transform = 
		tf::Transform(tf::Quaternion(_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w),
						tf::Vector3(_pose.position.x, _pose.position.y,_pose.position.z));
		broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),parent_frame, child_frame));
		// printf("map2odom_\n%f,%f,%f\n%f,%f,%f\n",transform.getRotation()[0],transform.getRotation()[1],transform.getRotation()[2],transform.getOrigin()[0],transform.getOrigin()[1],transform.getOrigin()[2]);									  
		loop_rate.sleep();
		ros::spinOnce();			
	}
	return 0;
}

