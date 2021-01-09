// #include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/core.hpp>
#include <opencv2/viz.hpp>

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
        cv::Mat map_prob;
        cv::Mat dst_mat;

        CostCube(double focal_len,double field_size,double resolution);
        CostCube(){}
        void reinitialize(double focal_len,double field_size,double resolution);
        cv::Mat calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points);
        cv::Mat calCostCubeByDistance(vector<geometry_msgs::Point> map_points);
        void processMapPts(const std::vector<geometry_msgs::Point> &pts,bool cal_occupied_only=false);
        bool Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,cv::Mat &visited,bool cal_occupied_only=false);
        float computeCostByDistance(const float distance);
        float dstFromVoxelToObstacle(vector<int> pos_id);
        float dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points);

private:
        double field_size = 0.15;
        double focal_len = 0.5;
        double resolution = 0.05;
        float occ_scale = 1.0;
        // int voxel_nxy;
        // int voxel_nz;
        int size[3];
        cv::Mat occupied_counter, visit_counter;
        vector<vector<int>> occupied_ind;
        int free_thresh = 5;
        int occupied_thresh = 5;
        double inscribed_radius_ = 0.01;
        double cost_scaling_factor = 1.0;
};