#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include "tf/transform_datatypes.h"//转换函数头文件

/** old code 
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
**/

static geometry_msgs::Pose _pose;

void PoseStampedCallback(geometry_msgs::PoseStamped pose){
	_pose = pose.pose;
}

void PoseCallback(geometry_msgs::Pose pose){
	_pose = pose;
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
	int rate = 1;
	if(argc > 4)
		rate = atoi(argv[4]);
	
	ros::Subscriber pose_sub = n.subscribe(pose_name, 5, PoseStampedCallback);
	tf::Transform transform = 
		tf::Transform(tf::Quaternion(_pose.orientation.x, _pose.orientation.y, _pose.orientation.z, _pose.orientation.w),
						tf::Vector3(_pose.position.x, _pose.position.y,_pose.position.z));
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(rate);
	while(ros::ok()){
		broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),parent_frame, child_frame));
		// printf("map2odom_\n%f,%f,%f\n%f,%f,%f\n",transform.getRotation()[0],transform.getRotation()[1],transform.getRotation()[2],transform.getOrigin()[0],transform.getOrigin()[1],transform.getOrigin()[2]);									  
		loop_rate.sleep();
		ros::spinOnce();			
	}
	return 0;
}
	/** old code
// 	map2odom_ = tf::Transform(tf::Quaternion(0,0,0,1),
// 		   	 							tf::Vector3(0,0,0));
// 	tf::TransformBroadcaster broadcaster;
//     ros::Subscriber gridpose_sub = n.subscribe("/grid_pose", 5, gridPoseCallback);
//     //发送消息的频率为2Hz， 1秒发2个，周期为500ms
//     ros::Rate loop_rate(100);
//     while(ok()){
//         broadcaster.sendTransform(tf::StampedTransform(map2odom_,ros::Time::now(),"map", "odom"));
// 		printf("map2odom_\n%f,%f,%f\n%f,%f,%f\n",map2odom_.getRotation()[0],map2odom_.getRotation()[1],map2odom_.getRotation()[2],map2odom_.getOrigin()[0],map2odom_.getOrigin()[1],map2odom_.getOrigin()[2]);									  
//          //靠sleep()函数保证连续两个消息之间的时间恰好为一个周期
// 	    ros::spinOnce();
// 		loop_rate.sleep();		
//     }    
**/
