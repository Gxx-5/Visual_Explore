#include"KDE.h"
#define PI 3.1415926f

KDE::KDE(int type,float h)
{
	// this->size=vec.size();
	this->type=type;
	// this->data_vec = vec;
	this->h=h;
}

double KDE::AvgRand(double min, double max)
{//Create 0-1 random.
 int minInteger = (int)(min*10000);
	int maxInteger = (int)(max*10000);
	int randInteger = rand()*rand();
	int diffInteger = maxInteger - minInteger;
	int resultInteger = randInteger % diffInteger + minInteger;
	return resultInteger/10000.0;
}
double KDE::GaussRand(double variance,double mean)
{//Create Gaussian random.
 double Temp;
 double u1,u2;
 u1=AvgRand(0,1);
 u2=AvgRand(0,1);
 if(u1<=1e-30){
  u1=1e-30;
 }
 Temp=sqrt(-2*(log(u1)))*sin(2*3.1415926*(u2));
 Temp=variance*Temp+mean; 
 return Temp;
}
void KDE::readData(char*fname)
{//Read data from the file.
 ifstream file(fname);
 if(!file.is_open())
 {
  cout<<"open error!"<<endl;
  exit(1);
 }
 file>>size;
 x=new float[size];
 for(int i=0;i<size;i++)
 {
  file>>x[i];
 }
 file.close();
}
void KDE::createRandData()
{//Create random data.
 for(int i=0;i<size;i++)
 {
  this->x[i]=(float)GaussRand(1.0,0.0);
 }
}
void KDE::sortData()
{//Sort the values of x.
 int i,j;
 float temp;
 bool done=false;
 j=1;
 while((j<size)&&(!done))
 {
  done=true;
  for(i=0;i<size-j;i++)
  {
   if(x[i]>x[i+1])
   {
	done=false;
	temp=x[i];
	x[i]=x[i+1];
	x[i+1]=temp;
   }
  }
  j++;
 }
}
float KDE::GaussKernel(float x)
{//Gaussian kernel
 float y=1/sqrt(2*PI)*exp(-0.5*x*x);
 return y;
}
float KDE::LaplaceKernel(float x)
{//Laplace kernel
 return 0.5f*exp(-fabs(x));
}
float KDE::EpaneKernel(float x)
{//Epanechnikov kernel
 float max=0;
 float t=1-x*x;
 if(max<t)
  max=t;
 return 0.75f*max;
}
float KDE::computeKDE(vector<float> data_vec)//Compute the values of KDE.
{
	//Compute mean and stdev
	double sum = std::accumulate(std::begin(data_vec), std::end(data_vec), 0.0);
	double mean =  sum / data_vec.size(); //均值 
	double accum  = 0.0;
	std::for_each (std::begin(data_vec), std::end(data_vec), [&](const double d) {
		accum  += (d-mean)*(d-mean);
	}); 
	double stdev = sqrt(accum/(data_vec.size()-1)); //方差

	int i,j;
	size = data_vec.size();
	float peak_value = -1.0f;
	float peak_data;
	float pre_value = 0.0f;
	for(i=0;i<size;i++){
		float cur_value = 0.0f;
		for(j=0;j<size;j++){
			switch(this->type){
				case 1:
					cur_value+=LaplaceKernel((data_vec[j]-data_vec[i])/h);
					break;
				case 2:
					cur_value+=EpaneKernel((data_vec[j]-data_vec[i])/h);
					break;
				default:
					cur_value+=GaussKernel((data_vec[j]-data_vec[i])/h);
			}
		}
		cur_value=cur_value/size;
		cur_value=cur_value/h;
		if(peak_value < 0 && cur_value < pre_value){
			peak_value = pre_value;
			peak_data = data_vec[i];
			break;
		}
		pre_value = cur_value;
	}
	double std_max = 0.0;
	if(stdev<std_max){
		double factor = (std_max-stdev)/std_max;
		peak_data = peak_data * (1-factor) + factor * mean;
	}
	// cout << "stdev: " << stdev << endl;
	return peak_data;
	// return stdev;
	// return mean;
}

void KDE::getDescribe(char cs[])
{//Get the discribe of the condition.
 switch(type)
 {
 case 1:
  sprintf(cs, "h=%0.2f, Laplace kernel",h);break;
 case 2:
  sprintf(cs, "h=%0.2f, Epanechnikov kernel",h);break;
 default:
  sprintf(cs, "h=%0.2f, Gaussian kernel",h);
 }
}