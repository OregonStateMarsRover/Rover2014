#include "node.h"

//need n which is learning rate
//error = y-t
//dw = n*e*y_2(1-y_2)y_1
//dv = n*d2*x
//d2 = sum of all outputs(w_kj*(e*y_2(1-y_2))y(1-y)) // the wight is preupdate

using namespace std;

node::node(int inputs, double bias){	
	srand(time(0));
	double w;
	this->numinputs = inputs;
	this->bias = bias;
	for(int i=0; i<inputs; ++i){
		w = rand()%100000;
		w /= 100000;
		this->w.push_back(w); 
	}
}
double node::getweight(int n){
	return this->w[n];
}

double node::activation_sum(vector<double> input){
	double sum = 0;
	for(unsigned int i=0; i<input.size(); ++i){
		sum += input[i] * this->w[i] + this->bias;
	}
	return sum;
}

double node::activation(vector<double> inputs){
	double a = this->activation_sum(inputs);
	this->output = 1/(1+pow(E, -a));
	if(this->output > .95){
		this->output = 1;
	}else if(this->output < .05){
		this->output = 0;
	}
	return this->output;
}

double node::activation_prime(double a){
	return pow(E, a)/pow(pow(E, a)+1, 2);
}
