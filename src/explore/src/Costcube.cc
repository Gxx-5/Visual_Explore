#include <queue>
#include "costcube.h"

vector<double> TransformPoint(Eigen::Matrix3d rotation,Eigen::Vector3d translation,vector<double> pos){
	Eigen::Matrix4d transform = Eigen::Matrix4d::Zero();
	transform.topLeftCorner<3,3>() = rotation;
	transform.topRightCorner<3,1>() = translation.transpose();
	transform(3,3) = 1;
	Eigen::Matrix<double,4,1> point(pos[0],pos[1],pos[2,1],1);
	Eigen::Matrix<double,4,1> point_trans = transform * point;
	return vector<double>{point_trans(0,0),point_trans(1,0),point_trans(2,0)};
}

CostCube::CostCube(vector<double> input_vec){
	// double* param[6]{&shooting_dst,&cam_width,&cam_height,&resolution,&dst_filter_factor,&cost_scaling_factor};
	for(int i=0;i<input_vec.size();++i){
		*param[i]=input_vec[i];
	}
	size[0] = int(shooting_dst / resolution); //x
	size[1] = int(cam_width / resolution);//y
	size[2] =int(cam_height / resolution); //z
	filter_triangle = getFilterTriangle();
	cam_posid = {0,int(size[1]/2),int(size[2]/2)};
	printParams();
}

CostCube::CostCube(string params_path){
	cout<<"get params from " << params_path << endl;
	cv::FileStorage read_params(params_path,cv::FileStorage::READ);

	shooting_dst = (double)read_params["shooting_dst"];
	cam_width = (double)read_params["cam_width"];
	cam_height = (double)read_params["cam_height"];
	resolution = (double)read_params["resolution"];
	kdtree_radius = (double)read_params["kdtree_radius"];
	kdtree_K = (double)read_params["kdtree_K"];
	cost_scaling_factor = (double)read_params["cost_scaling_factor"];
	trapezoid_hei = (double)read_params["trapezoid_hei"];
	trapezoid_len = (double)read_params["trapezoid_len"];
	dst_filter_factor = (double)read_params["dst_filter_factor"];
	read_params.release();

	size[0] =int(cam_width / resolution);  //x
	size[1] =int(shooting_dst / resolution);//y
	size[2] =int(cam_height / resolution); //z
	filter_triangle = getFilterTriangle();
	cam_posid = {int(size[0]/2) , 0 , int(size[2]/2)};
	printParams();
}

void CostCube::reinitialize(vector<double> input_vec){
	// double* param[6]{&shooting_dst,&cam_width,&cam_height,&resolution,&dst_filter_factor,&cost_scaling_factor};
	for(int i=0;i<input_vec.size();++i){
		*param[i]=input_vec[i];
	}
	size[0] = int(cam_width / resolution);
	size[1] = int(cam_height / resolution);
	size[2] = int(shooting_dst / resolution);
	filter_triangle = getFilterTriangle();
	printParams();
}

void CostCube::printParams()
{
	printf("Using params:\n");
	printf("shooting_dst:  %5.3f\n", shooting_dst);
	printf("cam_width:  %5.3f\n", cam_width);
	printf("cam_height:  %5.3f\n", cam_height);
	printf("resolution:  %5.3f\n", resolution);
	printf("dst_filter_factor:  %5.3f\n", dst_filter_factor);
	printf("cost_scaling_factor:  %5.3f\n", cost_scaling_factor);
	printf("size of costcube: [%d,%d,%d]\n", size[0],size[1],size[2]);
	printf("size of filter_triangle: [%d]\n", filter_triangle.size());
}

double CostCube::getresolution(){
	return resolution;
}

cv::Mat CostCube::calCostCubeByBresenham3D(vector<geometry_msgs::Point> map_points){
	map_prob = cv::Mat::zeros(3,size,CV_8UC1);
	if(map_points.size()==0)
		return map_prob;
	processMapPts(map_points);
	//cost = exp(-1.0 * cost_scaling_factor * (distance_from_obstacle – inscribed_radius)) * (costmap_2d::INSCRIBED_INFLATED_OBSTACLE – 1)
	int maxVisitNum = *max_element(visit_counter.begin<int>(),visit_counter.end<int>());
	for (int row = 0; row < size[0]; ++row)
	{
		for (int col = 0; col < size[1]; ++col)
		{
			for (int hei = 0;hei < size[2]; ++ hei){
				// continue;
				int visits = visit_counter.at<int>(row, col,hei);
				int occupieds = occupied_counter.at<int>(row, col,hei);                                

				if (occupieds){
					map_prob.at<uint>(row, col, hei) = 0;
				}
				else if (visits <= free_thresh)
				{
					map_prob.at<uint>(row, col, hei) = 255;
					//grid_map_proba.at<uint>(row, col) = 128;
				}
				else if(visits > occupied_thresh){
					map_prob.at<uint>(row, col, hei) = 0;
				}
				else{
					map_prob.at<uint>(row, col, hei) = int((1-1.0*visits/maxVisitNum)*255);
				}
			}
		}
	}
	return map_prob;
}

void CostCube::processMapPts(const std::vector<geometry_msgs::Point> &pts,bool cal_occupied_only){
	occupied_counter = cv::Mat::zeros(3,size,CV_32SC1);
	visit_counter = cv::Mat::zeros(3,size,CV_32SC1);
	// unsigned int end_id = start_id + n_pts;
	// for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)

	int invalid_num = 0;
	int distant_num = 0;
	for (unsigned int pt_id = 0; pt_id < pts.size(); ++pt_id)
	{
		double dst = sqrt(pow((pts[pt_id].x),2)+pow((pts[pt_id].y),2)+pow((pts[pt_id].z),2));
		if(dst > shooting_dst){
			continue;
			distant_num++;
		}
		if(!Bresenham3D(pts[pt_id], occupied_counter, visit_counter,cal_occupied_only))
			invalid_num++;
	}
	// cout << "size of occupied_ind after Bresenham3D algorithm: " << occupied_ind.size() << " , while size of map_points is " << pts.size() << endl;
	// cout << "Within " << pts.size() << " points , there are " << distant_num << " distant points and " << invalid_num << " invalid points" << endl;
}

bool CostCube::Bresenham3D(const geometry_msgs::Point &pt_pos, cv::Mat &occupied,cv::Mat &visited,bool cal_occupied_only){
	// https://gist.github.com/yamamushi/5823518#file-bresenham3d-L11
	// int x1 = int(size[0]/2);
	// int y1 = int(size[1]/2);
	// int z1 = int(size[2]/2);
	// int x2 = int((pt_pos.x - cam_pos.x)/resolution + size[0]/2);
	// int y2 = int((pt_pos.y - cam_pos.y)/resolution + size[1]/2);
	// int z2 = int((pt_pos.z - cam_pos.z)/resolution + size[2]/2);

	// int x1 = int(size[0]/2);
	// int y1 = int(size[1]/2);
	// int z1 = 0;
	vector<int> cam_posid{int(size[0]/2),int(size[1]/2),0};
	int x2 = int((pt_pos.x )/resolution + cam_posid[0]);
	int y2 = int((pt_pos.y )/resolution + cam_posid[1]);
	int z2 = int((pt_pos.z )/resolution  + cam_posid[2]);
	if (x2 < 0 || x2 >= size[0]||y2 < 0 || y2 >= size[1]||z2 < 0 || z2 >= size[2]){
		// cout << "Target index ["<< x2 << "," << y2 << "," << z2 << "] out of bound [" << size[0] << "," 
		//           << size[1] << "," << size[2]  << "](maximum) when calculating Bresenham3D" << endl;
		return false;
	}
	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(x2,y2,z2);
	occupied_ind.push_back(vector<int>{x2,y2,z2});
	if(cal_occupied_only) 
		return true;
	else{
		int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
		int point[3];
		
		point[0] = cam_posid[0];
		point[1] = cam_posid[1];
		point[2] = cam_posid[2];
		dx = x2 - cam_posid[0];
		dy = y2 - cam_posid[1];
		dz = z2 - cam_posid[2];
		x_inc = (dx < 0) ? -1 : 1;
		l = abs(dx);
		y_inc = (dy < 0) ? -1 : 1;
		m = abs(dy);
		z_inc = (dz < 0) ? -1 : 1;
		n = abs(dz);
		dx2 = l << 1;
		dy2 = m << 1;
		dz2 = n << 1;
		
		if ((l >= m) && (l >= n)) {
			err_1 = dy2 - l;
			err_2 = dz2 - l;
			for (i = 0; i < l; i++) {                
			++visited.at<int>(point[0], point[1], point[2]);
			if (err_1 > 0) {
				point[1] += y_inc;
				err_1 -= dx2;
			}
			if (err_2 > 0) {
				point[2] += z_inc;
				err_2 -= dx2;
			}
			err_1 += dy2;
			err_2 += dz2;
			point[0] += x_inc;
			}
		} else if ((m >= l) && (m >= n)) {
			err_1 = dx2 - m;
			err_2 = dz2 - m;
			for (i = 0; i < m; i++) {
			++visited.at<int>(point[0], point[1], point[2]);
			if (err_1 > 0) {
				point[0] += x_inc;
				err_1 -= dy2;
			}
			if (err_2 > 0) {
				point[2] += z_inc;
				err_2 -= dy2;
			}
			err_1 += dx2;
			err_2 += dz2;
			point[1] += y_inc;
			}
		} else {
			err_1 = dy2 - n;
			err_2 = dx2 - n;
			for (i = 0; i < n; i++) {
			++visited.at<int>(point[0], point[1], point[2]);
			if (err_1 > 0) {
				point[1] += y_inc;
				err_1 -= dz2;
			}
			if (err_2 > 0) {
				point[0] += x_inc;
				err_2 -= dz2;
			}
			err_1 += dy2;
			err_2 += dx2;
			point[2] += z_inc;
			}
		}
		++visited.at<int>(point[0], point[1], point[2]);
	}
	return true;
}

bool compare(const geometry_msgs::Point& p1,const geometry_msgs::Point& p2){
	if(p1.z<p2.z)
		return true;
	else
		return false;               
}
/*
//map points must be relative coords of key points in camera_link coordinate.
cv::Mat CostCube::calCostCubeByDistance(vector<geometry_msgs::Point> map_points){
	map_prob = cv::Mat::zeros(3,size,CV_32FC1);
	dst_mat = cv::Mat::zeros(3,size,CV_32FC1);
	occupied_ind.clear();
	if(map_points.size()==0){
		cout << "no map points received!" << endl;
		return map_prob;
	}
	// processMapPts(map_points,true);

	//calculate distance by map
	// map<double,geometry_msgs::Point> map_pts;//sort map_points by z coordinate.
	// for(vector<geometry_msgs::Point>::iterator it=map_points.begin();it!=map_points.end();++it){
	//         map_pts.insert(make_pair(it->z,*it));
	// }

	//calculate distance by kdtree
	// pointVec points;
	// for(vector<geometry_msgs::Point>::iterator it=map_points.begin();it!=map_points.end();++it){
	//         points.push_back(point_t{it->x,it->y,it->z});
	// }
	// KDTree tree(points);

	//calculate distance by PCL::KDTree
	//https://blog.csdn.net/qq_22170875/article/details/84786533?utm_medium=distribute.pc_relevant_t0.none-task-blog-OPENSEARCH-1.control&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-OPENSEARCH-1.control
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	for(vector<geometry_msgs::Point>::iterator it=map_points.begin();it!=map_points.end();++it){
		cloud->push_back(pcl::PointXYZ(it->x, it->y, it->z));
	}
	kdtree.setInputCloud(cloud); //设置需要建立kdtree的点云指针

	for (int row = 0; row < size[0]; ++row)
		for (int col = 0; col < size[1]; ++col)
			for (int hei = 0;hei < size[2]; ++ hei){
				if(hei < filter_triangle[row]){
					map_prob.at<float>(row, col, hei) = 0.0;
					continue;
				}
				// RTime rt("Distance calculation with priority queue");
				// TODO : Maybe need normalization?
				// float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei});
				// float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei},map_points);
				// float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei},map_pts);
				// float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei},tree);
				float dst = dstFromVoxelToObstacle(vector<int>{row,col,hei},kdtree,int(kdtree_K));
				dst_mat.at<float>(row, col, hei) = dst;
				if(dst < 0){//something wrong happen,dont change map_prob
					cout << "something wrong happen while calculating CostCube by Distance." << endl;
					return map_prob;
				}
				map_prob.at<float>(row, col, hei) = computeCostByDistance(dst);
				// map_prob.at<float>(row, col, hei) = dst; 
				// cout << "dst: " <<  dst << " " << ",cost : " <<  computeCostByDistance(dst) << endl;
			}
	return map_prob;
}
*/

cv::Mat CostCube::calCostCubeByDistance(Eigen::Matrix3d rotation,Eigen::Vector3d translation,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	map_prob = cv::Mat::zeros(3,size,CV_32FC1);
	dst_mat = cv::Mat::zeros(3,size,CV_32FC1);
	occupied_ind.clear();
	if(cloud->size()==0){
		cout << "no map points received!" << endl;
		return map_prob;
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;  //建立kdtree对象
	kdtree.setInputCloud(cloud); //设置需要建立kdtree的点云指针
	for (int row = 0; row < size[0]; ++row)
		for (int col = 0; col < size[1]; ++col)
			for (int hei = 0;hei < size[2]; ++ hei){
				// if(hei < filter_triangle[row]){
				// 	map_prob.at<float>(row, col, hei) = 0.0;
				// 	continue;
				// }
				vector<double> marker_pos{ (row - cam_posid[0]) * resolution,(col - cam_posid[1]) * resolution,(hei - cam_posid[2]) * resolution};
				vector<double> pos = TransformPoint(rotation,translation,marker_pos);
				float dst = dstFromVoxelToObstacle(pos,kdtree,int(kdtree_K));
				dst_mat.at<float>(row, col, hei) = dst;
				if(dst < 0){//something wrong happen,dont change map_prob
					cout << "something wrong happen while calculating CostCube by Distance." << endl;
					return map_prob;
				}
				map_prob.at<float>(row, col, hei) = computeCostByDistance(dst);
			}
	return map_prob;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id){
//Calculate average distance between current voxel and nearby obstacle. 
	if(pos_id.size()!=3){
		cout << "Wrong dim of voxel index has been input!";
		return -1;
	}
	vector<float> dst_vec;
	//calculate distance only between current voxel and the voxel which is occupied.
	int occ_n = occupied_ind.size();
	if(occ_n <= 0){
		cout <<"No obstacle points detected when calculate distance from voxel to obstacle! return -1" << endl;
		return -1;
	}
	for(uint i=0;i<occ_n;++i){
		dst_vec.push_back(resolution * sqrt(pow(occupied_ind[i][0]-pos_id[0],2)+pow(occupied_ind[i][1]-pos_id[1],2)+pow(occupied_ind[i][2]-pos_id[2],2)));
	}
	sort(dst_vec.begin(),dst_vec.end());
	float dst_thresh = (dst_vec.back() - dst_vec.front()) * dst_filter_factor +dst_vec.front() ;
	// cout << occ_n << " " << dst_thresh << " "<<  dst_vec.back() << " " << dst_vec.front() << endl;
	float dst = 0.0;
	int i;
	for(i=0;dst_vec[i]<=dst_thresh;++i){
		dst +=  dst_vec[i];
	}
	return dst/i;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points){
//Calculate average distance between current voxel and all map points in the field of view. 
	if(pos_id.size()!=3){
		cout << "Wrong dim of voxel index has been input!";
		return -1;
	}
	// map<int,float> dst_map;
	priority_queue<float, vector<float>, greater<float> > dst_queue;
	// vector<int> cam_posid{int(field_size / resolution),int(field_size / resolution),0};
	vector<int> cam_posid{int(size[0]/2),int(size[1]/2),0};
	float x = (pos_id[0] -cam_posid[0]) * resolution;
	float y = (pos_id[1] - cam_posid[1]) * resolution;
	float z = (pos_id[2] - cam_posid[2]) * resolution;
	// {
	//         RTime rt("dst_queue push");
		for(uint i=0;i<map_points.size();++i){
			float dst = sqrt(pow(map_points[i].x - x , 2) + pow(map_points[i].y - y , 2) + pow(map_points[i].z-z , 2));
			// dst_map.insert(make_pair(i,dst));
			dst_queue.push(dst);
		}
	// }
	// sort(dst_map.begin(),dst_vec.end());

	// float dst_thresh = (dst_vec.back() - dst_vec.front()) * dst_filter_factor +dst_vec.front() ;
	// // cout << occ_n << " " << dst_thresh << " "<<  dst_vec.back() << " " << dst_vec.front() << endl;
	// float dst = 0.0;
	// int i;
	// for(i=0;dst_vec[i]-dst_thresh>-0.001;++i){
	//         dst +=  dst_vec[i];
	//         if(dst<0){
	//                 cout << "dst<0! dst_vec[i]:" << dst_vec[i] << endl;
	//                 for(vector<float>::iterator it=dst_vec.begin();it!=dst_vec.end()-1;++it){
	//                         cout << *it << endl;
	//                 }
	//                 cout << *(dst_vec.end()-1) << endl;
	//                 cout << "dst_vec.back(): " <<  dst_vec.back() << endl;
	//                 cout << "dst_vec.front(): " <<  dst_vec.front() << endl;
	//                 cout << "dst_thresh: " <<  dst_thresh << endl;
	//                 int nth;
	//                 cin >> nth;
	//         }
	// }
	int i=0;
	float dst=0;
	// {
	//         RTime rt("dst_queue pop");
		while(i<int(dst_queue.size()*dst_filter_factor)){
			// dst+=dst_vec[i];
			dst+=dst_queue.top();
			++i;
			dst_queue.pop();
		}
	// }
	return dst/i;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,map<double,geometry_msgs::Point> map_pts){
//Calculate average distance between current voxel and all map points in the field of view. 
	if(pos_id.size()!=3){
		cout << "Wrong dim of voxel index has been input!";
		return -1;
	}
	vector<int> cam_posid{int(size[0]/2),int(size[1]/2),0};
	float x = (pos_id[0] -cam_posid[0]) * resolution;
	float y = (pos_id[1] - cam_posid[1]) * resolution;
	float z = (pos_id[2] - cam_posid[2]) * resolution;

	int step = int(map_pts.size()*dst_filter_factor);
	// int search_id = step;
	auto iter = find_if(map_pts.begin(),map_pts.end(),Nearest_MapValue(z));
	if(iter == map_pts.end()){
		cout << "cannot find nearby map point with [" << x << "," << y << "," << z << "]." << endl;
	}
	else{
		int ind = distance(map_pts.begin(),iter);
		// if(search_id<step){
		//         iter = map_pts.begin();
		//         advance(iter,step);
		//         //search_id = step;
		// }
		if (ind > map_pts.size() - step){
			iter = map_pts.end();
			advance(iter,-step);
			// search_id = map_pts.size() - step;         
		}
		else if(ind < step/2){
			iter = map_pts.begin();                    
		}
		else{
			advance(iter,-step/2);
		}
	}
	int i;float dst=0;
	// for(i = search_id-step;i<search_id+step;++i){
	for(i=0;i<step;++i,++iter){//calculate nearest points distance in z coordinate.
		if(iter==map_pts.end()){
			cout << "sth wrong happen while calculating nearby obstacle distance. i= " << i << " step= " << step << endl;
		}
		dst += sqrt(pow((*iter).second.x- x , 2) + pow((*iter).second.y - y , 2) + pow((*iter).second.z-z , 2));
	}
	return dst/i;
}

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,KDTree tree){
	// RTime rt("kdtree");
	if(pos_id.size()!=3){
		cout << "Wrong dim of voxel index has been input!";
		return -1;
	}
	double x = (pos_id[0] -cam_posid[0]) * resolution;
	double y = (pos_id[1] - cam_posid[1]) * resolution;
	double z = (pos_id[2] - cam_posid[2]) * resolution;
	point_t pt{x,y,z};
	auto ptVec = tree.neighborhood_points(pt,kdtree_radius);
	if(ptVec.size()==0){
		auto nearest_pt = tree.nearest_point(pt);
		float dst = sqrt(pow(nearest_pt[0] - x , 2) + pow(nearest_pt[1] - y , 2) + pow(nearest_pt[2]-z , 2));
		return dst;
	}
	float ave_dst = 0;
	for (point_t pt : ptVec) {
		float dst = sqrt(pow(pt[0] - x , 2) + pow(pt[1] - y , 2) + pow(pt[2]-z , 2));
		ave_dst+=dst;
	}
	return ave_dst/ptVec.size();
}

float CostCube::dstFromVoxelToObstacle(vector<double> pos,pcl::KdTreeFLANN<pcl::PointXYZ> kdtree,int K){        
	// double x = (pos_id[0] -cam_posid[0]) * resolution + cam_pos[0];
	// double y = (pos_id[1] - cam_posid[1]) * resolution + cam_pos[1];
	// double z = (pos_id[2] - cam_posid[2]) * resolution + cam_pos[2];
	pcl::PointXYZ searchPoint{pos[0],pos[1],pos[2]};
	float dst=0;
	if(K>0){
		std::vector<int> pointIdxNKNSearch(K);  //保存每个近邻点的索引
		std::vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找点之间的欧式距离平方
		kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		for(vector<float>::iterator it=pointNKNSquaredDistance.begin();it!=pointNKNSquaredDistance.end();++it){
			dst+=sqrt(*it);                  
		}
		return dst/K;
	}
	else{
		std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
		std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
		kdtree.radiusSearch(searchPoint, kdtree_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
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

float CostCube::dstFromVoxelToObstacle(vector<int> pos_id,vector<geometry_msgs::Point> map_points,int tag){
//Calculate average distance between current voxel and all map points in the field of view. 
	if(pos_id.size()!=3){
		cout << "Wrong dim of voxel index has been input!";
		return -1;
	}
	vector<float> dst_vec;
	// vector<int> cam_posid{int(field_size / resolution),int(field_size / resolution),0};
	vector<int> cam_posid{int(size[0]/2),int(size[1]/2),0};
	float x = (pos_id[0] -cam_posid[0]) * resolution;
	float y = (pos_id[1] - cam_posid[1]) * resolution;
	float z = (pos_id[2] - cam_posid[2]) * resolution;
	for(uint i=0;i<map_points.size();++i){
		float dst = sqrt(pow(map_points[i].x - x , 2) + pow(map_points[i].y - y , 2) + pow(map_points[i].z-z , 2));
		dst_vec.push_back(dst);
	}
	sort(dst_vec.begin(),dst_vec.end());

	// float dst_thresh = (dst_vec.back() - dst_vec.front()) * dst_filter_factor +dst_vec.front() ;
	// // cout << occ_n << " " << dst_thresh << " "<<  dst_vec.back() << " " << dst_vec.front() << endl;
	// float dst = 0.0;
	// int i;
	// for(i=0;dst_vec[i]-dst_thresh>-0.001;++i){
	//         dst +=  dst_vec[i];
	//         if(dst<0){
	//                 cout << "dst<0! dst_vec[i]:" << dst_vec[i] << endl;c
	//                 for(vector<float>::iterator it=dst_vec.begin();it!=dst_vec.end()-1;++it){
	//                         cout << *it << endl;
	//                 }
	//                 cout << *(dst_vec.end()-1) << endl;
	//                 cout << "dst_vec.back(): " <<  dst_vec.back() << endl;
	//                 cout << "dst_vec.front(): " <<  dst_vec.front() << endl;
	//                 cout << "dst_thresh: " <<  dst_thresh << endl;
	//                 int nth;
	//                 cin >> nth;
	//         }
	// }
	int i=0;
	float dst=0;
	while(i<int(dst_vec.size()*dst_filter_factor)){
		dst+=dst_vec[i];
		++i;
	}
	return dst/i;
}

float CostCube::computeCostByDistance(const float distance)
{
	float cost;
	if(distance < inscribed_radius_){
		cost = 1.0;
	}
	else{
		cost = exp(-1.0 * cost_scaling_factor * (distance - inscribed_radius_));
		// cost = 255 * cost;//(unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
	}
	// cout << "compute process : dst: " << distance << " cost: " <<  cost << endl;
	return cost;
}

map<int,int> Bresenham2D(int x0,int y0,int x1,int y1){
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;
	map<int,int> triangle;
	triangle[x0]=y0;
	while(x0 != x1 && y0 != y1){
		e2 = err;
		if (e2 > -dx)
		{
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy)
		{
			err += dx;
			y0 += sy;
		}
		triangle[x0]=y0;
		// cout << "x,y: " << x0 << "," << y0 << endl;
	}
	return triangle;
}

map<int,int> CostCube::getFilterTriangle(){
	int x0 = 0;
	int y0 = int(trapezoid_hei / resolution);
	int x1 = int((cam_width-trapezoid_len)/2/resolution)-1;
	int y1 = 0;
	map<int,int> triangle1 = Bresenham2D(x0,y0,x1,y1);
	for(int i=0;i<int(trapezoid_len/resolution);++i){
		triangle1[x1+i] = 0;
	}

	x0 =  int((cam_width+trapezoid_len)/2/resolution)-1;
	y0 = 0;
	x1 = int(cam_width/resolution)-1;
	y1 = int(trapezoid_hei / resolution);
	map<int,int> triangle2 = Bresenham2D(x0,y0,x1,y1);

	triangle1.insert(triangle2.begin(),triangle2.end());
	if(triangle1.size()!=int(cam_width/resolution)){
		cout << "Actual triangle size: " << triangle1.size() << " Expected triangle size: " << int(cam_width/resolution) << endl;
		// for(map<int,int>::iterator it=triangle1.begin();it!=triangle1.end();++it){
		//         cout << it->first << " " << it->second << endl;
		// }
	}       
	return triangle1;
}