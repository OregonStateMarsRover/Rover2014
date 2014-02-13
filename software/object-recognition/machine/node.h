#define MAXCLASS 8
#define E (2.7182818284590452353602874713526624977572470937L )

#include <iostream>
#include <cmath>
#include <gsl/gsl_math.h>
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <sstream>

using namespace std;

class node{
	double w1, w2, bias;
	
	public:
	node(float, float, float);
	void adjust_w1(float);
	float get_w1();
};
struct surf_descriptor{
	int x;
	int y;
	float descs[128];
};



int vector2class(vector<int>);
vector<int> class2vector(int);
vector<vector<surf_descriptor> > get_surf_from_file(string);
float activation(float);
float activation_prime(float);

