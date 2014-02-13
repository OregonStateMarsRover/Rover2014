#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <sstream>

//opencv stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//include my stuff
#include "node.h"

#include "constants.h"

using namespace std;
using namespace cv;

struct surf_descriptor{
	int x;
	int y;
	double descs[128];
};

struct neural_network{
	vector<double> inputs;
	vector<node> hidden;
	vector<node> outputs;
};


int vector2class(vector<int>);
vector<int> class2vector(int);
vector<vector<surf_descriptor> > get_surf_from_file(string);
int rgbcat(int r, int g, int b);
vector<double> run_nn(neural_network nn);
neural_network nn_init(vector<double> inputs);
