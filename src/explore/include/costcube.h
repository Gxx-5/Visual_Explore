// #include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>
#include <opencv2/viz.hpp>
#include <ctime>
#include <algorithm>

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

//double _shooting_dst,double _cam_width,double _cam_height,double _resolution,double _dst_filter_factor = 0.1,double _cost_scaling_factor = 10.0
class CostCube
{
public:
        CostCube(vector<double> input_vec);
        CostCube(){}
        void reinitialize(vector<double> input_vec);
        void printParams();	
	
        double getresolution();
        cv::Mat calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points);
        cv::Mat calCostCubeByDistance(vector<geometry_msgs::Point> map_points);
        void processMapPts(const std::vector<geometry_msgs::Point> &pts,bool cal_occupied_only=false);
        bool Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,cv::Mat &visited,bool cal_occupied_only=false);
        float computeCostByDistance(const float distance);
        float dstFromVoxelToObstacle(vector<int> pos_id);
	float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points);
        float dstFromVoxelToObstacle(vector<int> pos_id,map<int,pair<double,geometry_msgs::Point>> map_pts);

private:
        double shooting_dst = 0.5;// the farest distance camera can catch in world coordinate. default : 0.5
        double cam_width = 0.64;//the width of camera field size. default : 0.64 if kinect
        double cam_height = 0.48;//the height of camera field size. default : 0.48 if kinect        
        double resolution = 0.05;// the resolution of costcube. default : 0.05
        double dst_filter_factor = 1.0;//the proportion to keep nearby points calculated by distance. default : 0.1
        double cost_scaling_factor = 10.0;//the weight of distance in cost calculation (same as costmap). default : 10.0
        int size[3];

        cv::Mat map_prob;
        cv::Mat dst_mat;
        cv::Mat occupied_counter, visit_counter;
        vector<vector<int>> occupied_ind;
        int free_thresh = 5;
        int occupied_thresh = 5;
        double inscribed_radius_ = 0.01;
};

class RTime{
public:
	RTime(string _func_name){
		start = clock();
		func_name = _func_name;
	}
	~RTime(){
		end = clock();
		cout << func_name <<  " has cost " << (double)(end-start)/CLOCKS_PER_SEC << "seconds" << endl;
	}
private:
	clock_t start,end;
	string func_name;	
};

class Nearest_MapValue{
public:
	Nearest_MapValue(const float _digit):digit(_digit){}
       bool operator ()(const map<int,pair<double,geometry_msgs::Point>>::value_type &pair)
       {
            return pair.second.first > digit;
       }
private:
        const float digit;
};