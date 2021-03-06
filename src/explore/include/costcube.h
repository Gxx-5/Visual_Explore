// #include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>
#include <opencv2/viz.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ctime>
#include <algorithm>
#include "KDTree.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/kdtree/kdtree_flann.h>  //kdtree近邻搜索
#include "pcl/search/impl/organized.hpp"
#include "pcl/surface/impl/convex_hull.hpp"
#include <pcl/point_types.h>  //点类型相关定义
#include <KDE.h>

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

/**
 * KDTree、Annoy、HNSW原理和使用方法介绍
 * https://my.oschina.net/u/4410617/blog/4354043
**/

//double _shooting_dst,double _cam_width,double _cam_height,double _resolution,double _dst_filter_factor = 0.1,double _cost_scaling_factor = 10.0

vector<double> TransformPoint(const Eigen::Matrix3d &rotation,const Eigen::Vector3d &translation,const vector<double> &pos);

class CostCube
{
public:
	CostCube(vector<double> input_vec);
	CostCube(string params_path);
	CostCube(){}
	void reinitialize(vector<double> input_vec);
	void printParams();	

	double getresolution();	
	cv::Mat calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points);
	cv::Mat calCostCubeByDistance(const Eigen::Matrix3d &rotation,const Eigen::Vector3d &translation,const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	void processMapPts(const std::vector<geometry_msgs::Point> &pts,bool cal_occupied_only=false);
	bool Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,cv::Mat &visited,bool cal_occupied_only=false);
	void processOccludedArea(cv::Mat& map,const vector<vector<int>>& obs_id);
	float computeCostByDistance(const float &distance);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const Eigen::Matrix3d &rotation,const Eigen::Vector3d &translation,const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
	// float dstFromVoxelToObstacle(vector<double> pos,pcl::KdTreeFLANN<pcl::PointXYZ> tree,int K);
	float dstFromVoxelToObstacle(const vector<double> &pos,const pcl::KdTreeFLANN<pcl::PointXYZ> &kdtree,const int &K);

	cv::Mat calCostCubeByDistance(vector<geometry_msgs::Point> map_points);
	// cv::Mat calCostCubeByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	float dstFromVoxelToObstacle(vector<int> pos_id);
	float dstFromVoxelToObstacle(vector<int> pos_id,KDTree tree);
	float dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points);        
	float dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points,int tag);
	float dstFromVoxelToObstacle(vector<int> pos_id,map<double,geometry_msgs::Point> map_pts);
	// float dstFromVoxelToObstacle(vector<int> pos_id,map<int,pair<double,geometry_msgs::Point>> map_pts);
	map<int,int> getFilterTriangle();

	int size[3];
	vector<int> cam_posid;

private:
	double shooting_dst = 0.5;// the farest distance camera can catch in world coordinate. default : 0.5
	double cam_width = 0.64;//the width of camera field size. default : 0.64 if kinect
	double cam_height = 0.48;//the height of camera field size. default : 0.48 if kinect        
	double resolution = 0.05;// the resolution of costcube. default : 0.05
	double kdtree_radius = 0.1;
	double kdtree_K = 0;
	double cost_scaling_factor = 10.0;//the weight of distance in cost calculation (same as costmap). default : 10.0
	double trapezoid_hei = 0.0;//the height of camera field view trapezoid. default : 0.0	
	double trapezoid_len = 0.0;//the narrow side length of camera field view trapezoid. default : 0.0
	double dst_filter_factor = 0.1;//the proportion to keep nearby points calculated by distance. default : 0.1
	double* param[10]{&shooting_dst,&cam_width,&cam_height,&resolution,&kdtree_radius,&kdtree_K,
                                          &cost_scaling_factor,&trapezoid_hei,&trapezoid_len,&dst_filter_factor};
  // double* param[10]={&shooting_dst,&cam_width,&cam_height,&resolution,&kdtree_radius,&cost_scaling_factor,&trapezoid_hei,&trapezoid_len,&dst_filter_factor};

	KDE kde = KDE(0);
	double cam_pos[3];
	map<int,int> filter_triangle;
	cv::Mat map_prob;
	cv::Mat dst_mat;
	cv::Mat occupied_counter, visit_counter;
	vector<vector<int>> occupied_ind;
	int free_thresh = 5;
	int occupied_thresh = 5;
	double inscribed_radius_ = 0.0;
	double max_cost = 1.0;
};

class RTime{
public:
	RTime(string _func_name){
		start = clock();
		func_name = _func_name;
	}
	~RTime(){
		end = clock();
		cout << func_name <<  " has cost " << (double)(end-start)/CLOCKS_PER_SEC << " seconds" << endl;
	}
private:
	clock_t start,end;
	string func_name;	
};

class Nearest_MapValue{
public:
	Nearest_MapValue(const float _digit):digit(_digit){}
//bool operator ()(const map<int,pair<double,geometry_msgs::Point>>::value_type &pair)
  bool operator ()(const map<double,geometry_msgs::Point>::value_type &pair)
  {
    return pair.first > digit;
  }
private:
  const float digit;
};

