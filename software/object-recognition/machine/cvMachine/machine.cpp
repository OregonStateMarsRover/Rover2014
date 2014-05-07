#define ISIZE 20
#define HSIZE 20
#define OSIZE 10
#define LAYERSIZE 30
#define MAX_ITERATIONS 1000
#define MIN_ERROR .01
#define LEARNING_RATE .1


#include <vector>
#include <iostream>

using namespace std;

#include <opencv2/opencv.hpp> //include all cpp

using namespace cv;

/*
 * Creates a Simple Mat that contains a random list of inputs
 */
void create_inputs(Mat &inputs){
	inputs.create(1, ISIZE, 1);
	randu(inputs, Scalar::all(0), Scalar::all(30));	
}

vector<int> int2bin(int n){
	vector<int> bin;
	while(n != 1){
		if(n%2 == 1){
			bin.push_back(1);
			n--;
		}else{
			bin.push_back(0);
		}
		n /= 2;
	}
	//the output must always be 10 bits
	for(;bin.size() < 10;){
		bin.push_back(0);
	}
	return bin;
}


void create_training_params(CvANN_MLP_TrainParams &training_params){
	CvTermCriteria term_crit = cvTermCriteria( CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, MAX_ITERATIONS, MIN_ERROR);
	training_params = CvANN_MLP_TrainParams(term_crit, CvANN_MLP_TrainParams::BACKPROP , LEARNING_RATE,  LEARNING_RATE);
}


void train_x2(){
	//initalize a neural network
	CvANN_MLP nn = CvANN_MLP();

	//create the neural network 
	vector<int> layer_size;
	layer_size.push_back(ISIZE);
	layer_size.push_back(HSIZE);
	layer_size.push_back(OSIZE);
	nn.create(Mat(layer_size), CvANN_MLP::SIGMOID_SYM, 0, 0);
	
	Mat inputs;
	create_inputs(inputs);
	
	CvANN_MLP_TrainParams training_params;
	create_training_params(training_params);
	
	Mat outputs(inputs.rows, 1, 0);
	for(int i=0; i > inputs.rows; ++i){
		outputs.at<int>(i, 0, 0) = inputs.at<int>(i, 0)*inputs.at<int>(i, 0);
	}

	//train the neural network
	int iters =  nn.train(inputs, outputs, Mat(), Mat(), training_params);
	
	cout << "Ran " << iters << "items" << endl;
		
}

int main(){
		train_x2();
        return 0;
}
