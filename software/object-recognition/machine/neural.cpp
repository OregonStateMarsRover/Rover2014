#include "neural.h"

using namespace std;
using namespace cv;

/*
 * Converts a vector to a classification number
 */
int vector2class(vector<double> v){
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
vector<double> class2vector(int classification){
	vector<double> v;
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

/*
 * this is a function to try and test if the neural network can find x2 bellow 256(8 bit)
 */
void train_x2(){
	vector<vector<double> > inputs;
	vector<vector<double> > outputs;
	for(int i = 0; i < 256; i+=3){
		vector<double> x;	
		x.push_back(i);
		inputs.push_back(x);
		vector<double> y;
		int bin = i*i;
		while(bin > 0){
			if(bin%2 == 1){
				y.push_back(1);
				bin--;
			}
			bin /= 2;
		}
		y.push_back(i*i);
		outputs.push_back(y);
	}
	neural_network nn(inputs.size(), 8);
	nn.train(inputs, outputs);
	for(int i=1; i < 256; i+=2){
		vector<double> input;
		input.push_back(i);
		vector<double> outs = nn.run(input);
		double total = 0;
		for(int n = 0; n < 8; ++n){
			if(outs[n] == 1){
				total += pow(2, 8-n);
			}
		}
		//cout << "Test:" << i << ":" << total << endl;
	}
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
	train_x2();	
	return 0;
}
