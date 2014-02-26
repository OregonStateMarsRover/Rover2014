#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <sstream>

//include my stuff
#include "node.h"

#include "constants.h"


class neural_network{
	//the layers
	vector<double> inputs;
	vector<node> hidden;
	vector<node> outputs;
	
	//the output of the layers
	vector<double> y1;
	vector<double> y2;
	vector<double> error;

	int hiddensize;
	int outputsize;
	
public:
	neural_network(unsigned int numinputs, unsigned int numoutputs); 
	vector<double> run(vector<double> inputs);
	vector<double> calc_error(vector<double> expected);
	void backpropagation();
	double train(vector<vector<double> > inputs, vector<vector<double> > expected);
private:
	double calc_mse(vector<double>);
};
