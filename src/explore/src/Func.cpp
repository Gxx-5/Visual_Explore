#include<iostream>
#include<vector>
#include<cmath>
#include <opencv2/core/core.hpp>

using namespace std;

vector<float> TransformCoordinate(vector<float> pos,vector<float> quat,vector<float> point){
        float qx=quat[0],qy=quat[1],qz=quat[2],qw=quat[3];
        float QuatRotation[3][3]={
                {1-2*pow(qy,2)-2*pow(qz,2) , 2*qx*qy+2*qw*qz , 2*qx*qz-2*qw*qy},
                {2*qx*qy-2*qw*qz , 1-2*pow(qx,2)-2*pow(qz,2) , 2*qy*qz+2*qw*qx},
                {2*qx*qz+2*qw*qy , 2*qy*qz-2*qw*qx , 1-2*pow(qx,2)-2*pow(qy,2)}
        };
        float Rotation[4][4]={
                QuatRotation[0][0],QuatRotation[0][1],QuatRotation[0][2],pos[0],
                QuatRotation[1][0],QuatRotation[1][1],QuatRotation[1][2],pos[1],
                QuatRotation[2][0],QuatRotation[2][1],QuatRotation[2][2],pos[2],
                0,0,0,1
        };
        float Coord[4][1]={point[0],point[1],point[2],1};
        cv::Mat RotationMat(4,4,CV_32F,Rotation);
        cv::Mat CoordMat(4,1,CV_32F,Coord);
        cv::Mat CoordTransMat = RotationMat*CoordMat;
        return vector<float>{CoordTransMat.at<float>(0,0),CoordTransMat.at<float>(1,0),CoordTransMat.at<float>(2,0)};
}

cv::Mat Euler2Quaternion(float *angle)
{
        float heading = angle[0];
        float attitude = angle[1];
        float bank = angle[2];

        float c1 = cos(heading/2);
        float s1 = sin(heading/2);
        float c2 = cos(attitude/2);
        float s2 = sin(attitude/2);
        float c3 = cos(bank/2);
        float s3 = sin(bank/2);
        float c1c2 = c1*c2;
        float s1s2 = s1*s2;
        float w =c1c2*c3 - s1s2*s3;
        float x =c1c2*s3 + s1s2*c3;
        float y =s1*c2*c3 + c1*s2*s3;
        float z =c1*s2*c3 - s1*c2*s3;
        float q[4] = {w,x,y,z};
        cv::Mat ret(4,1,CV_32FC1,q);
        return ret.clone();
}

#include<iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

using namespace std;
int main()
{
//    //1.p1 world position
    double p1yaw=0;
    double p1x=1;
    double p1y=1;
    Eigen::AngleAxisd rotzp1(p1yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d  t1= Eigen::Vector3d(p1x,p1y, 0);
    Eigen::Quaterniond q1=Eigen::Quaterniond(rotzp1);
    //cout<<"q1.vec()"<<q1.vec()<<endl;
    cout<<"p1 eular angle"<<(180/M_PI)*q1.matrix().eulerAngles(0,1,2)<<endl;
    
    //2. t12 value
    double t12yaw=45;
    double t12x=1;
    double t12y=1;
    Eigen::AngleAxisd rott12(t12yaw*M_PI/180, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d  t12= Eigen::Vector3d(t12x,t12y, 0);
    Eigen::Quaterniond q12=Eigen::Quaterniond(rott12);
     cout<<"t12 rotation eular angle "<<(180/M_PI)*q12.matrix().eulerAngles(0,1,2)<<endl;
  //  cout<<"q12.vec()"<<q12.vec()<<endl;
     
//    3.calculate p2 world positon
    Eigen::Quaterniond q2=q1*q12;
   // cout<<" q2 vector " <<q2.vec()<<endl;
    Eigen::Vector3d t2=t1+q1.toRotationMatrix()*t12;
    cout<<"t2="<<t2<<endl;
    cout<<"t2 eular angle "<<(180/M_PI)*q2.matrix().eulerAngles(0,1,2)<<endl;
}
