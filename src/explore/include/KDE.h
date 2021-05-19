//KDE.h
#include<cmath>
#include<iostream>
#include<fstream>
#include<vector>
#include <numeric>
#include <algorithm>

using namespace std;

class KDE
{
public:
        float* x;//x[0...size-1]
        float* y;//y[0...size-1]
        // vector<float> data_vec;
        float h;
        int size;
        int type;//0-Gaussian,1-Laplace,2-Epanechnikov

public:
        KDE(int type,float h = 0.5f);
        void createRandData();
        void readData(char*fname);
        void sortData();
        float computeKDE(vector<float> data_vec);
        void getDescribe(char cs[]);

private:
        double AvgRand(double min, double max);
        double GaussRand(double variance,double mean);
        float GaussKernel(float x);
        float LaplaceKernel(float x);
        float EpaneKernel(float x);
};
