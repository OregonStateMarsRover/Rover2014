#define MAXCLASS 8

#include <iostream>
#include <cmath>
#include <gsl/gsl_math.h>
#include <vector>

using namespace std;

class node{
	double w1, w2, bias;
	
	public:
	node(float, float, float);
	void adjust_w1(float);
	float get_w1();
};


int vector2class(vector<int>);
vector<int> class2vector(int);
