/**
代码思路：
1.输入视野概率图（预处理--PreProcess()）
2.基于预处理后的概率通行图进行采样（SampleDirection() ）
3.设定一个阈值，筛选出可通行方向（FilterDirection() ）
**/
#include "FilterDirection.h"

using namespace std;

cv::Mat FilterDirection::PreProcess(cv::Mat costcube_map){

}

vector<coord> FilterDirection::SampleDirection(cv::Mat costcube_map){

}

vector<float> FilterDirection::getDirection(){

}
