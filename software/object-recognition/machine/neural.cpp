#include "neural.h"

using namespace std;
using namespace cv;

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

int rgbcat(int r, int g, int b){
	return (r*1000000+g*1000+b);
}


/*
 * This will read the surf descriptor data file that we generated with
 * a python script. this data is of the format
 * image_id<int> x<int> y<int> 1...128<double>
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

neural_network nn_init(vector<double> inputs){
	neural_network nn;
	nn.inputs = inputs;
	int numinputs = inputs.size();
	//start by setting up the hidden and output layers for the neural network
	for(int i=0; i<HIDDEN; ++i){
		node n(numinputs, .01);
		nn.hidden.push_back(n);
	}
	for(int i=0; i<MAXCLASS; ++i){
		node n(numinputs, .01);
		nn.outputs.push_back(n);
	}
	return nn;
}

vector<double> run_nn(neural_network nn){
	vector<double> hout;
	for(unsigned int i=0; i<nn.hidden.size(); ++i){
		double out = nn.hidden[i].activation(nn.inputs);
		hout.push_back(out);
	}
	vector<double> output;
	for(unsigned int i=0; i<nn.outputs.size(); ++i){
		double out = nn.outputs[i].activation(hout);
		output.push_back(out);	
	}
	return output;
}


int main(){
	//TODO add support for looping through all of the image files
	vector<double> inputs;	
	//take an image and create a new vector with the merged rgb values
	Mat image;
	string imagepath = "images/hockeypuck/image001.png";
	image = imread(imagepath, CV_LOAD_IMAGE_COLOR);
	if(!image.data){
		cout <<  "Could not open or find the image" << std::endl ;
		return -1;
	}
	int r,g,b;
	double rgbc;
	for(int row=0; row<image.rows; ++row){
		unsigned char *data = image.ptr(row);
		for(int col=0;col<image.cols; ++col){
			//read the bgr sequental values from data
			b = *data++;
			g = *data++;
			r = *data++;
			rgbc = (double)rgbcat(r, g, b);
			//scale concated rgb values from 0 to 1
			inputs.push_back(rgbc/256256256);
		}
	}	
	//now that we know the size of the input layer we can create the nural network
	neural_network nn = nn_init(inputs);
	vector<double> output = run_nn(nn);	
	return 0;
}
