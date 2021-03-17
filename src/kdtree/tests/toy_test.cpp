#include <iostream>
#include <vector>

// #include "KDTree.hpp"
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/kdtree/kdtree_flann.h>  //kdtree近邻搜索
#include "pcl/search/impl/organized.hpp"
#include "pcl/surface/impl/convex_hull.hpp"
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>

// using point_t = std::vector< double >;
// using pointVec = std::vector< point_t >;

// point_t pt(2);

using namespace std;

float TestKdtree(vector<int> coord,pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,float K){
	pcl::PointXYZ searchPoint{coord[0],coord[1],coord[2]};
	float dst=0;
    float fMin = pow(10,-6);
	if( fabs(K-int(K)) < fMin ){//K是一个整数，代表寻找kdtree临近的K个点
		std::vector<int> pointIdxNKNSearch(K);  //保存每个近邻点的索引
		std::vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找点之间的欧式距离平方
		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		for(vector<float>::iterator it=pointNKNSquaredDistance.begin();it!=pointNKNSquaredDistance.end();++it){
			dst+=sqrt(*it);                  
		}
		return dst/K;
	}
	else{//K是一个浮点数，代表寻找kdtree附近距离为K的所有点
		std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
		std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
		kdtree.radiusSearch(searchPoint, K, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        cout << "RadiusSearch Points num: " << pointIdxRadiusSearch.size() << endl;
		int count=0;
		for(vector<float>::iterator it=pointRadiusSquaredDistance.begin();it!=pointRadiusSquaredDistance.end();++it){
			dst+=sqrt(*it);
			count++;
		}
		if(count==0){
			std::vector<int> pointIdxNKNSearch(1); 
			std::vector<float> pointNKNSquaredDistance(1);
			kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance); 
			return sqrt(pointNKNSquaredDistance[0]);
		}
		return dst/count;
	}
}

void VisualizePclCloud(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_Cloud){
    //5双视窗口
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("filter Viewer"));
    viewer->initCameraParameters();
    int v1(0), v2(0);//视口编号在这里设置两个视口
    //5.1原始点云窗口
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("original", 10, 10, "v1 text", v1);
    viewer->addPointCloud<pcl::PointXYZ>(vec_Cloud[0], "sample cloud1", v1);
    viewer->addCoordinateSystem(1.0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
    //5.2滤波窗口
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("after filtered", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZ>(vec_Cloud[1], "sample cloud2", v2);
    viewer->addCoordinateSystem(1.0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");
    while (!viewer->wasStopped())
    {
    viewer->spinOnce(100);  //刷新
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree1;  //建立kdtree对象
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;  //建立kdtree对象
    vector<int> point{0,0,0}; 
    float radius = 7000;
    float res = 0.1;
    clock_t start,end;

    srand((int)time(0));
	// for(int i=0;i<1000;i++){        
    //     float radius = 1.0;
    //     float x = (rand() / double(RAND_MAX) * 2 - 1) * radius;
    //     float y = (rand() / double(RAND_MAX) * 2 - 1) * radius;
    //     float z = (rand() / double(RAND_MAX) * 2 - 1) * radius;
	// 	cloud1->push_back(pcl::PointXYZ(x,y,z));
	// }
    float side_len = 2.0*2;
    for(float x=-side_len/2;x<side_len/2;x+=res){
        for(float y=-side_len/2;y<side_len/2;y+=res){
            for(float z=-side_len/2;z<side_len/2;z+=res){
                cloud1->push_back(pcl::PointXYZ(x,y,z));
            }
        }
    }
	kdtree1.setInputCloud(cloud1); //设置需要建立kdtree的点云指针

    start = clock();
    float dst1 = TestKdtree(point,kdtree1,radius);
    end = clock();
    cout << "dst1: " << dst1 << " spent time : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
    
    // 可视化点云
    // pcl::visualization::CloudViewer viewer1("Cloud1 Viewer");
    // viewer1.showCloud(cloud1);
    // while (!viewer1.wasStopped()) {
    // }

    srand((int)time(0));
	// for(int i=0;i<5000;i++){        
    //     float radius = 5.0;
    //     float x = (rand() / double(RAND_MAX) * 2 - 1) * radius;
    //     float y = (rand() / double(RAND_MAX) * 2 - 1) * radius;
    //     float z = (rand() / double(RAND_MAX) * 2 - 1) * radius;
	// 	cloud2->push_back(pcl::PointXYZ(x,y,z));
	// }
    side_len = 7.0*2;
    for(float x=-side_len/2;x<side_len/2;x+=res){
        for(float y=-side_len/2;y<side_len/2;y+=res){
            for(float z=-side_len/2;z<side_len/2;z+=res){
                cloud2->push_back(pcl::PointXYZ(x,y,z));
            }
        }
    }
	kdtree2.setInputCloud(cloud2); //设置需要建立kdtree的点云指针

    start = clock();
    float dst2 = TestKdtree(point,kdtree2,radius);
    end = clock();
    cout << "dst2: " << dst2 << " spent time : " << (double)(end - start) / CLOCKS_PER_SEC << endl;
    // 可视化点云
    // pcl::visualization::CloudViewer viewer2("Cloud2 Viewer");
    // viewer2.showCloud(cloud2);
    // while (!viewer2.wasStopped()) {
    // }

    VisualizePclCloud(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>{cloud1,cloud2});


/**
    pointVec points;
    point_t pt;

    pt = {0.0, 0.0,0.0};
    points.push_back(pt);
    pt = {1.0, 0.0,0.0};
    points.push_back(pt);
    pt = {0.0, 1.0,0.0};
    points.push_back(pt);
    pt = {1.0, 1.0,0.0};
    points.push_back(pt);
    pt = {0.5, 0.5,0.0};
    points.push_back(pt);

    KDTree tree(points);

    std::cout << "nearest test\n";
    pt = {0.8, 0.2,0.0};
    auto res = tree.nearest_point(pt);
    for (double b : res) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    /**
    std::cout << "going down the tree\n";

    for (auto b : point_t(*tree)) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    for (auto b : point_t(*tree->left)) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    for (auto b : point_t(*tree->right)) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    for (auto b : point_t(*tree->left->left)) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    for (auto b : point_t(*tree->right->left)) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    std::cout << "printing nbh\n";

    pt = {.0, .5};

    
    auto res2 = tree.neighborhood_points(pt, .55);
    
    for (point_t a : res2) {
        for (double b : a) {
            std::cout << b << " ";
        }
        std::cout << '\n';
    }
**/
    
    return 0;
}
