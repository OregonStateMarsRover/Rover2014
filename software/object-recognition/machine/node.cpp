#include "node.h"

using namespace std;

node::node(float w1, float w2, float bias){
	this->w1 = w1;
	this->w2 = w2;
	this->bias = bias;
}
void node::adjust_w1(float weight){
	this->w1 -= weight;
}
float node::get_w1(){
	return this->w1;
}

/*
 * Converts a vector to a classification number
 */
int vector2class(vector<int> v){
	int counter = 0;
	while(v.at(counter) == 0 && counter < MAXCLASS){
		counter++;
	}
	if(counter == MAXCLASS && v.at(MAXCLASS) == 0){
		return 0;
	}
	return counter+1;
}

/*
 * Converts a classification number to a vector
 * Assumes that classification is valid
 */
vector<int> class2vector(int classification){
	vector<int> v;
	for(int i=0; i < MAXCLASS; ++i){
		v.push_back(classification == i);
	}
	return v;
}

/*
 * This will read the surf descriptor data file that we generated with
 * a python script. this data is of the format
 * image_id<int> x<int> y<int> 1...128<float>
 *
 */

vector<vector<surf_descriptor> > get_surf_from_file(string filename){
	ifstream f(filename.c_str(), ios::in);
	if(!f.is_open()){
		cout << "couldnt open file" << endl;
		exit(EXIT_FAILURE);
	}
	// level 1 is the image, level two is the descriptors 
	vector<vector<surf_descriptor> > imagev;
	string line;
	int img = 0;	
	while(getline(f, line)){	
		istringstream iis(line);
		int image, x, y;
		if(!(iis >> image >> x >> y)){
			cout << "Bad Data format" << endl;
			continue;
		}
		if(img == image){	
			vector<surf_descriptor> descv;
			imagev.push_back(descv);
			++img;
		}
		surf_descriptor item;
		item.x = x;
		item.y = y;
		for(int i=0; i < 128; ++i){
			iis >> item.descs[i];
		}
		imagev[image].push_back(item);	
	}
	return imagev;
}

float activation(float a){
	return 1/(1+pow(E, -a));
}

float activation_prime(float a){
	return pow(E, a)/pow(pow(E, a)+1, 2);
}

int main(){	
	vector<vector<surf_descriptor> > imagev = get_surf_from_file("surf_desc.dat");
	cout << "imported descriptors for " << imagev.size() << " images" << endl;
	return 0;
}
