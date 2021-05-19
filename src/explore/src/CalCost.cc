#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>
#include "costcube.h"
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

// double shooting_dst = 1.0;
// double cam_width = 0.64;
// double cam_height = 0.48;
// double dst_filter_factor = 0.1;
// double cost_scaling_factor = 10.0;
double resolution;//resolution of CostCube
vector<double> input_vec;//initial param of CostCube 
double obs_cost;
int obs_count;
//相机内参
float f_u = 718.856;
float f_v = 718.856;
float c_u = 607.1928;
float c_v = 185.2157;
std::string cloud_name = "/ros_cloud";
std::string pose_name = "/cam_pose";
Eigen::Matrix3d Rwc;
Eigen::Vector3d Twc;
double cam_pos[3];
vector<int> cam_posid;
cv::Mat image_raw;

vector<geometry_msgs::Point> map_points;//store map points within the field of vision
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//Publish
ros::Publisher vis_pub,vis_text_pub,horizontal_text_pub,vertical_text_pub;
ros::Publisher costcloud_pub;
ros::Publisher horizontal_cloud_pub,vertical_cloud_pub;
ros::Publisher pt_pub;
image_transport::Publisher image_pub;

std::mutex mpt_mutex;
std::mutex pose_mutex;

void VisualizeCostCube(const cv::Mat &cost_map);
void testColorfunc();
void parseParams(int argc, char **argv);
void printParams();
void preProcess(const sensor_msgs::PointCloud::ConstPtr& pt_cloud);
void preProcess2(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud);
void preProcessRosMsg2Pcl(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud2);
void PublishMapPoints(vector<geometry_msgs::Point> map_points,ros::Publisher publisher);
void poseCallback(const geometry_msgs::PoseStamped &pose);
void poseStampedCallback(const geometry_msgs::PoseStamped &pose);
bool detectObstacle(cv::Mat cost_map);
void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
void showObstacle2D(const cv::Mat &cost_map,const float &thre_cost);
vector<uchar> getColor(const float& value);

template <typename Func,typename Var,typename Ret>
void getTime(Func func,Var var,Ret ret){
		clock_t stime = clock();
		ret = func(var);
		clock_t etime = clock();
		cout << "Time spent: " << (double)(etime-stime)/CLOCKS_PER_SEC << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "explore");
	// ros::start();
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	parseParams(argc, argv);
	printParams();
	// CostCube COSTCUBE(input_vec);
	string params_path;
	n.getParam("/explore_node/params_path",params_path);
	CostCube COSTCUBE(params_path);
	cam_posid = COSTCUBE.cam_posid;

	// initialize
	resolution = COSTCUBE.getresolution();
	Eigen::Quaterniond quat(0,0,0,1);
	Rwc = quat.toRotationMatrix();
	Twc(0,0) = Twc(1,0) = Twc(2,0) = 0;

	//Subscribe
	ros::Subscriber pose_sub = n.subscribe(pose_name, 10, &poseStampedCallback);
	// ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud>(cloud_name,1,boost::bind(&preProcess,_1,map_points));
	// ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud>(cloud_name,1,&preProcess);
	// ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_name,1,&preProcess2);
	ros::Subscriber cloud_sub = n.subscribe<sensor_msgs::PointCloud2>(cloud_name,1,&preProcessRosMsg2Pcl);
	ros::Subscriber image_sub = n.subscribe<sensor_msgs::Image>("debug_image",1,&imageCallBack);

	//Publish
	// pt_pub = n.advertise<sensor_msgs::PointCloud>("pt_cloud",10);
	// vis_pub = n.advertise<visualization_msgs::MarkerArray>("costcube",10);
	vis_text_pub = n.advertise<visualization_msgs::MarkerArray>("cost_text",10);
	horizontal_text_pub = n.advertise<visualization_msgs::MarkerArray>("horizontal_text", 10);
	vertical_text_pub = n.advertise<visualization_msgs::MarkerArray>("vertical_text", 10);
	costcloud_pub = n.advertise<sensor_msgs::PointCloud>("costcloud", 10);
	horizontal_cloud_pub = n.advertise<sensor_msgs::PointCloud>("horizontal_costcloud", 10);
	vertical_cloud_pub = n.advertise<sensor_msgs::PointCloud>("vertical_costcloud", 10);
	image_pub = it.advertise("image_modified",1);
	ros::Publisher vel_pub= n.advertise<geometry_msgs::Twist>("cmd_vel",1,true);	
	ros::Publisher filtered_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);

	ros::Rate loop_rate(100);
	while(ros::ok()){
		{
			ros::Time stime = ros::Time::now();
			unique_lock<mutex> mlock(mpt_mutex);
			// PublishMapPoints(map_points,pt_pub);
			{
				// RTime rtime("Main Program");
				// ros::Time stime = ros::Time::now();
				// cv::Mat costcube_map = COSTCUBE.calCostCubeByDistance(map_points);
				cv::Mat costcube_map = COSTCUBE.calCostCubeByDistance(Rwc,Twc,map_cloud);
				// pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = COSTCUBE.filterCloud(Rwc,Twc,map_cloud);
				// sensor_msgs::PointCloud2 ros_filtered_cloud;				
				// pcl::toROSMsg(*filtered_cloud,ros_filtered_cloud);
				// ros_filtered_cloud.header.frame_id = "map";
				// ros_filtered_cloud.header.stamp = ros::Time::now();
				// filtered_cloud_pub.publish(ros_filtered_cloud);
				
				// ros::Time etime = ros::Time::now();
				// cout << "CalCostCube time spent : " << (etime - stime).toSec() << endl;

				// if(detectObstacle(costcube_map)){
				// 	cout << "Detected obstacle ahead!!!! Stop automatically." << endl;
				// 	geometry_msgs::Twist twist;
				// 	twist.linear.x=0;
				// 	twist.linear.y=0;
				// 	twist.linear.z=0;
				// 	twist.angular.x=0;
				// 	twist.angular.y=0;
				// 	twist.angular.z=0;
				// 	vel_pub.publish(twist);
				// }
				VisualizeCostCube(costcube_map);
				showObstacle2D(costcube_map,1.15);
			}
			ros::Time etime = ros::Time::now();
			cout << "Total time spent : " << (etime - stime).toSec() << endl << endl;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 1;
}

void imageCallBack(const sensor_msgs::ImageConstPtr& msg){
	try
	{
		cv_bridge::CvImagePtr cv_ptr; 
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
		cv_ptr->image.copyTo(image_raw);
		// cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
}

void fillRect(cv::Mat& image,cv::Point center,int radius,vector<uchar> color){
	int max_row = image.rows - 1;
	int max_col = image.cols - 1;
	int left_row = max(center.y - radius, 0);
	int left_col = max(center.x - radius, 0);
	int right_row = min(center.y + radius, max_row);
	int right_col = min(center.x + radius, max_col);
	for(int row=left_row;row<=right_row;row++){
		for(int col=left_col;col<=right_col;col++){
			image.at<cv::Vec3b>(row,col)[0] = color[2];
			image.at<cv::Vec3b>(row,col)[1] = color[1];
			image.at<cv::Vec3b>(row,col)[2] = color[0];
		}
	}
}

void showObstacle2D(const cv::Mat &cost_map,const float &thre_cost){
	if(!image_raw.data)
		return;		
	// float thre_cost = 1.19;
	float height = 0.1;

	// cv::Mat empty_img(3, image_raw.size(), CV_8UC3, cv::Scalar::all(0));
	cv::Mat image_modified = image_raw.clone();
	// cv::Mat empty_img = cv::Mat::ones(image_raw.rows,image_raw.cols,CV_8UC3)*255;
	// cout << image_raw.cols << " " << image_raw.rows << endl;
	// cout << empty_img.cols << " " << empty_img.rows << endl;
	// cout << empty_img.type() << endl;
	// cout << image_raw.type() << endl;
	// cv::Mat image_modified = cv::imread("/home/gxx/WorkSpace/Visual_Explore_ws/000000.png",cv::IMREAD_COLOR);
	cv::Mat K = cv::Mat::zeros(3,3,CV_32FC1);
	K.at<float>(0, 0) = f_u;
	K.at<float>(1, 1) = f_v;
	K.at<float>(0, 2) = c_u;
	K.at<float>(1, 2) = c_v;
	K.at<float>(2, 2) = 1.0;
	// cout << K << endl;

	for (int row = 0; row < cost_map.size[0]; ++row)
		for (int col = 0; col < cost_map.size[1]; ++col)
			for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				float cur_cost = cost_map.at<float>(row, col, hei);
				if(cur_cost <= thre_cost){
					vector<double> marker_pos{(row - cam_posid[0]) * resolution,(col - cam_posid[1]) * resolution,(hei - cam_posid[2]) * resolution};
					// vector<double> pos = TransformPoint(Rwc,Twc,marker_pos);
					cv::Mat point_mat = cv::Mat::zeros(3,1,CV_32FC1);
					point_mat.at<float>(0, 0) = marker_pos[0];
					point_mat.at<float>(1, 0) = -marker_pos[1]+height;
					point_mat.at<float>(2, 0) = marker_pos[2];
					cv::Mat res = K*point_mat;
					// cout << "cam_posid: \n" << cam_posid[0] << cam_posid[1] << cam_posid[2] << endl;
					// cout << "cost_map.size: \n"  << cost_map.size[0] << cost_map.size[1] << cost_map.size[2] << endl;
					// cout << "point_mat: \n" << point_mat << endl;
					// cout << "res: " << res << endl;
					if(res.at<float>(2, 0)<=0)
						continue;
					int v = res.at<float>(0, 0) / res.at<float>(2, 0);
					int u = res.at<float>(1, 0) / res.at<float>(2, 0);
					// cout << v << " " << u << endl << endl;

					cv::Point centerCircle1(v, u);
					int radiusCircle = 30;
					cv::Scalar colorCircle1(0, 0, int(255*cur_cost/thre_cost)); // (B, G, R)
					// vector<uchar> colorCircle1{0, 0, uchar(255*cur_cost/thre_cost)}; // (B, G, R)
					vector<uchar> color  = getColor(cur_cost*100000);
					// vector<uchar> color{255,255,0};

					// cout << cur_cost << endl;
					// cout << int(color[0]) << " " << int(color[1]) << " " << int(color[2]) << endl;
					int thicknessCircle1 = -1;
					// cv::circle(image_modified, centerCircle1, radiusCircle, colorCircle1, thicknessCircle1);
					fillRect(image_modified,centerCircle1,radiusCircle,color);

					// cout << image_modified.size() << endl;
					// cout << image_modified.cols << " " <<  image_modified.rows << endl;
					// if(u < 0 || v < 0 || u> image_modified.cols || v > image_modified.rows)
					// 	continue;
					// image_modified.at<int>(v,u) = 0;
				}
			}
			cv::addWeighted(image_raw,0.7,image_modified,0.3,0,image_modified);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_modified).toImageMsg();
			image_pub.publish(msg);
			// bool res_w = cv::imwrite("/home/gxx/WorkSpace/Visual_Explore_ws/image_modified.jpg",image_modified);
			// cv::imwrite("/home/gxx/WorkSpace/Visual_Explore_ws/image_raw.jpg",image_raw);
}

bool detectObstacle(cv::Mat cost_map){
	int count = 0;
	float max_cost=0;
	for (int row = 0; row < cost_map.size[0]; ++row)
		for (int col = 0; col < cost_map.size[1]; ++col)
			for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				float cur_cost = cost_map.at<float>(row, col, hei);
				if(cur_cost>max_cost){
					max_cost=cur_cost;
				}
				if(cur_cost > obs_cost){
					count ++;
				}
			}
	cout << "obs count : " << count << " max_cost : " << max_cost  <<  ", threshold count : "  << obs_count << endl;
	if(count>obs_count){
		return true;
	}
	else{
		return false;
	}
}

void preProcess(const sensor_msgs::PointCloud::ConstPtr& pt_cloud){	
	unique_lock<mutex> mlock(mpt_mutex);
	unique_lock<mutex> plock(pose_mutex);
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
	// cout << map_points.size() << endl;
}

void preProcess2(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud2){	
	unique_lock<mutex> mlock(mpt_mutex);
	unique_lock<mutex> plock(pose_mutex);
	map_points.clear();
	sensor_msgs::PointCloud::Ptr pt_cloud(new sensor_msgs::PointCloud);
	sensor_msgs::convertPointCloud2ToPointCloud(*pt_cloud2, *pt_cloud);
	for(int i=0;i<pt_cloud->points.size();++i){
		geometry_msgs::Point point;
		Eigen::Vector3d pos(pt_cloud->points[i].x,pt_cloud->points[i].y,pt_cloud->points[i].z);
		pos = Rwc.inverse()*(pos - Twc); // transform key points from "map" coordinate to "camera_link" coordinate and get relative coord.
		point.x = pos(0,0);
		point.y = pos(1,0);
		point.z = pos(2,0);
		map_points.push_back(point);
	}
	cout << "map_points.size: " << map_points.size() << endl;
}

void preProcessRosMsg2Pcl(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud2){
	pcl::fromROSMsg(*pt_cloud2, *map_cloud);
}

void poseCallback(const geometry_msgs::Pose &pose){
	unique_lock<mutex> plock(pose_mutex);
	Eigen::Quaterniond quat;
	quat.x() = pose.orientation.x;
	quat.y() = pose.orientation.y;
	quat.z() = pose.orientation.z;
	quat.w() = pose.orientation.w;

	Rwc = quat.toRotationMatrix();
	Twc(0,0) = pose.position.x;
	Twc(1,0) = pose.position.y;
	Twc(2,0) = pose.position.z;
}

void poseStampedCallback(const geometry_msgs::PoseStamped &pose){
	Eigen::Quaterniond quat;
	quat.x() = pose.pose.orientation.x;
	quat.y() = pose.pose.orientation.y;
	quat.z() = pose.pose.orientation.z;
	quat.w() = pose.pose.orientation.w;

	Rwc = quat.toRotationMatrix();
	Twc(0,0) = pose.pose.position.x;
	Twc(1,0) = pose.pose.position.y;
	Twc(2,0) = pose.pose.position.z;

	cam_pos[0] = pose.pose.position.x;
	cam_pos[1] = pose.pose.position.y;
	cam_pos[2] = pose.pose.position.z;
}

// void mainProcess(vector<geometry_msgs::Point> map_points){
// 	costcube_map = COSTCUBE.calCostCubeByDistance(map_points);
// 	VisualizeCostCube(costcube_map);
// }

void PublishMapPoints(vector<geometry_msgs::Point> map_points,ros::Publisher publisher){
	int size = map_points.size();
	cout << "PublishMapPoints size: " << size << endl;
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

vector<uchar> getColor(const float& value){
	vector<uchar> startColor{0,255,0};
	vector<uchar> endColor{255,255,0};
	float max_val = 2.0f;
	float min_val = 1.0f; 
	float interval = max_val - min_val;

	if(value >= max_val)
		return endColor;
	else if(value <= min_val)
		return startColor;

	uchar r_gap=endColor[0]-startColor[0];
	uchar g_gap=endColor[1]-startColor[1];
	uchar b_gap=endColor[2]-startColor[2];
	
	// Reset the colors to the starting position
	uchar rStart=startColor[0];
	uchar gStart=startColor[1];
	uchar bStart=startColor[2];	

	// float step = (value - 255)/255;
	// int step = value;
	uchar r = rStart + r_gap * value / interval;
	uchar g = gStart + g_gap * value / interval;
	uchar b = bStart + b_gap * value / interval;
	// float a = value / 255;
	// cout << r << " " << g << " " << b << endl;
	return vector<uchar>{r,g,b};
	// return vector<int>{(int)(rStart+rStep*step+0.5),(int)(gStart+gStep*step+0.5),(int)(bStart+bStep*step+0.5)};
}

void VisualizeCostCube(const cv::Mat& cost_map){
	if(cost_map.empty()){
		cout << "CostCube map is empty." << endl;
		return;
	}
	// visualization_msgs::MarkerArray markerArr;
	visualization_msgs::MarkerArray markerTextArr;
	visualization_msgs::MarkerArray markerHorizontalSliceTextArr;
	visualization_msgs::MarkerArray markerVerticalSliceTextArr;
/*
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
	marker.scale.x = 0.8 * resolution;
	marker.scale.y = 0.8 * resolution;
	marker.scale.z = 0.8 * resolution;
	marker.color.a = 0.1; // Don't forget to set the alpha!
*/
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
	marker_text.scale.z = 0.3 * resolution;
	marker_text.color.a = 1.0; // Don't forget to set the alpha!!

	sensor_msgs::PointCloud cloud;
	int pt_num = cost_map.size[0]*cost_map.size[1]*cost_map.size[2];
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = "map";//填充 PointCloud 消息的头：frame 和 timestamp．
	cloud.points.resize(pt_num);//设置点云的数量．
 
	//增加信道 "intensity" 并设置其大小，使与点云数量相匹配．
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(pt_num);

	sensor_msgs::PointCloud horizontal_cloud = cloud;
	int slice_pt_num = cost_map.size[0]*cost_map.size[2];
	horizontal_cloud.points.resize(slice_pt_num);
	horizontal_cloud.channels[0].values.resize(slice_pt_num);
	sensor_msgs::PointCloud vertical_cloud = cloud;
	slice_pt_num = cost_map.size[1]*cost_map.size[2];
	vertical_cloud.points.resize(slice_pt_num);
	vertical_cloud.channels[0].values.resize(slice_pt_num);
	//使用虚拟数据填充 PointCloud 消息．同时，使用虚拟数据填充 intensity 信道．

	// vector<int> cam_posid = {0,int(cost_map.size[1] / 2),int(cost_map.size[2] / 2)};
	// cout << int(field_size / resolution) << " " << cost_map.size[0] << endl;
	int marker_id = 0;
	int i=0,j = 0,k=0,l=0;
	for (int row = 0; row < cost_map.size[0]; ++row){
		for (int col = 0; col < cost_map.size[1]; ++col){
			for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				float cur_cost = cost_map.at<float>(row, col, hei);
				vector<uchar> color  = getColor(cur_cost);
				// marker.pose.position.x = camera_pose.position.x + (row - cam_posid[0]) * resolution;
				// marker.pose.position.y = camera_pose.position.y + (col - cam_posid[1]) * resolution;
				// marker.pose.position.z = camera_pose.position.z + (hei - cam_posid[2]) * resolution;
				vector<double> marker_pos{(row - cam_posid[0]) * resolution,(col - cam_posid[1]) * resolution,(hei - cam_posid[2]) * resolution};
				vector<double> pos = TransformPoint(Rwc,Twc,marker_pos);
				marker_text.pose.position.x = pos[0];
				marker_text.pose.position.y = pos[1];
				marker_text.pose.position.z = pos[2];
				marker_text.color.r =  color[0];
				marker_text.color.g = color[1];
				marker_text.color.b = color[2];
				marker_text.id = marker_id++;
				// markerArr.markers.push_back(marker);				
				// marker_text.pose = marker.pose;
				marker_text.text = to_string(cur_cost).substr(0,4);
				marker_text.id = marker_id - 1;
				markerTextArr.markers.push_back(marker_text);
				if(col==cam_posid[1]){
					markerHorizontalSliceTextArr.markers.push_back(marker_text);
				}
				if(row==cam_posid[0]){
					markerVerticalSliceTextArr.markers.push_back(marker_text);
				}

				cloud.points[i].x = pos[0]; //](row - cam_posid[0]) * resolution  + cam_pos[0];
				cloud.points[i].y = pos[1]; // (col - cam_posid[1]) * resolution + cam_pos[1];
				cloud.points[i].z = pos[2]; //  (hei - cam_posid[2]) * resolution + cam_pos[2];
				cloud.channels[0].values[i] = int(cur_cost*100);
				if(col==cam_posid[1]){
					horizontal_cloud.points[j] = cloud.points[i];
					horizontal_cloud.channels[0].values[j] = cloud.channels[0].values[i];
					j++;
				}
				if(row==cam_posid[0]){
					vertical_cloud.points[k] = cloud.points[i];
					vertical_cloud.channels[0].values[k] = cloud.channels[0].values[i];
					k++;
				}
				i++;
			}
		}
	}
	// vis_pub.publish(markerArr);
	vis_text_pub.publish(markerTextArr);
	horizontal_text_pub.publish(markerHorizontalSliceTextArr);
	vertical_text_pub.publish(markerVerticalSliceTextArr);
	/*
	int i=0,j = 0,k=0;
	for (int row = 0; row < cost_map.size[0]; ++row)
		for (int col = 0; col < cost_map.size[1]; ++col)
			for (int hei = 0;hei < cost_map.size[2]; ++ hei){
				vector<double> marker_pos{ (row - cam_posid[0]) * resolution,(col - cam_posid[1]) * resolution,(hei - cam_posid[2]) * resolution};
				vector<double> pos = TransformPoint(Rwc,Twc,marker_pos);
				cloud.points[i].x = pos[0]; //](row - cam_posid[0]) * resolution  + cam_pos[0];
				cloud.points[i].y = pos[1]; // (col - cam_posid[1]) * resolution + cam_pos[1];
				cloud.points[i].z = pos[2]; //  (hei - cam_posid[2]) * resolution + cam_pos[2];
				float cur_cost = cost_map.at<float>(row, col, hei);
				cloud.channels[0].values[i] = int(cur_cost*100);				

				if(hei==cam_posid[2]){
					horizontal_cloud.points[j] = cloud.points[i];
					horizontal_cloud.channels[0].values[j] = cloud.channels[0].values[i];
					j++;
				}
				if(col==cam_posid[1]){
					vertical_cloud.points[k] = cloud.points[i];
					vertical_cloud.channels[0].values[k] = cloud.channels[0].values[i];
					k++;
				}
				i++;
			}	
	*/
	costcloud_pub.publish(cloud);
	horizontal_cloud_pub.publish(horizontal_cloud);
	vertical_cloud_pub.publish(vertical_cloud);
}

void parseParams(int argc, char **argv)
{
	int arg_id = 1;
	if (argc > arg_id)
	{
		cloud_name = argv[arg_id++];
	}
	if (argc > arg_id)
	{
		pose_name = argv[arg_id++];
	}
	if (argc > arg_id)
	{
		obs_count = atoi(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		obs_cost = atof(argv[arg_id++]);
	}
	while(argc > arg_id){
		input_vec.push_back(atof(argv[arg_id++]));
	}
	// if (argc > arg_id)
	// {
	// 	shooting_dst = atof(argv[arg_id++]);
	// }
	// if (argc > arg_id)
	// {
	// 	field_size = atof(argv[arg_id++]);
	// }
	// if (argc > arg_id)
	// {
	// 	resolution = atof(argv[arg_id++]);
	// }
	// if (argc > arg_id)
	// {
	// 	dst_filter_factor = atof(argv[arg_id++]);
	// }
	// if (argc > arg_id)
	// {
	// 	cost_scaling_factor = atof(argv[arg_id++]);
	// }

}

void printParams()
{
	// printf("Using params:\n");
	// printf("shooting_dst: %f\n", shooting_dst);
	// printf("field_size: %f\n", field_size);
	// printf("resolution: %f\n", resolution);
	printf("cloud_topic_name: %s\n", cloud_name.c_str());
	printf("pose_topic_name: %s\n", pose_name.c_str());
}

void testColorfunc(ros::Publisher vis_pub,ros::Publisher vis_text_pub){
	float resolution = 10.0/255;
	visualization_msgs::MarkerArray markerArr;
	visualization_msgs::MarkerArray markerTextArr;
	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_text;
	marker.header.frame_id = "camera_link";
	marker.header.stamp = ros::Time::now();
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
	marker_text.scale.z = resolution;
	marker_text.color.a = 1.0; // Don't forget to set the alpha!!

	for(int i =0; i<255;++i){
		marker.pose.position.x = -5.0+i*resolution;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		vector<uchar> color  = getColor(i);
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
