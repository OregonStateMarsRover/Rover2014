#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include "constants.h"
using namespace std;

class node{
	int numinputs;	
	double bias;
	double output;
	
	public:
	vector<double> w;

	node(int inputs, double bias);
	double getweight(int n);
	double activation_sum(vector<double> input);
	double activation(vector<double>);
	double activation_prime(double);
};
