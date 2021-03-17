#include <opencv2/core/core.hpp>

typedef float coord[2];

class FilterDirection{
public:
        FilterDirection(){}
        cv::Mat PreProcess(cv::Mat costcube_map);
        vector<coord> SampleDirection(cv::Mat costcube_map);
        vector<float> getDirection();

};