#include "neural_network.h"

neural_network::neural_network(unsigned int numinputs, unsigned int numoutputs){
	this->hiddensize = HIDDEN;
	this->outputsize = numoutputs;
	//start by setting up the hidden and output layers for the neural network
	for(int i=0; i<HIDDEN; ++i){
		node n(numinputs, 1);
		this->hidden.push_back(n);
	}
	for(unsigned int i=0; i<numoutputs; ++i){
		node n(numinputs, 1);
		this->outputs.push_back(n);
	}
}

vector<double> neural_network::run(vector<double> inputs){
	this->y1.clear();
	this->y2.clear();
	this->inputs = inputs;
	for(unsigned int i=0; i<this->hidden.size(); ++i){
		double out = this->hidden[i].activation(inputs);
		this->y1.push_back(out);
	}
	for(unsigned int i=0; i<this->outputs.size(); ++i){
		double out = this->outputs[i].activation(this->y1);
		this->y2.push_back(out);	
	}
	return this->y2;
}

vector<double> neural_network::calc_error(vector<double> expected){
	this->error.clear();
	for(unsigned int i=0; i < this->outputs.size(); ++i){
		this->error.push_back(this->y2[i] - expected[i]);
	}
	return this->error;
}

void neural_network::backpropagation(){
	vector<vector<double> > w_ho; //weights from hidden to output
	vector<vector<double> > w_ih; //weights from input to hidden
	//calculate the weigth changes between the hidden and output layer
	for(unsigned int i=0; i < this->outputs.size(); ++i){
		vector<double> nodew;
		w_ho.push_back(nodew);
		for(unsigned int x=0; x < this->hidden.size(); ++x){
			float dw = N*this->error[i]*this->y2[i]*(1-this->y2[i])*this->y1[x];
			w_ho[i].push_back(this->outputs[i].w[x] + dw);
			//cout << "h2o:" << dw << endl;
		}
	}
	//now the weights between input and hidden layer
	for(unsigned int z=0; z < hidden.size(); ++z){
		vector<double> nodew;
		w_ih.push_back(nodew);
		for(unsigned int n=0; n < this->inputs.size(); ++n){
			float d2 = 0;
			for(unsigned int m=0; m < this->outputs.size(); ++m){
				d2 += this->outputs[m].w[z]*this->error[m]*
					this->y2[m]*(1-this->y2[m])*this->y1[z]*
					(1-this->y1[z]);
			}
			float dv = d2*N*this->inputs[n];
			cout << dv << endl;
			w_ih[z].push_back(this->hidden[z].w[n] + dv);
			//cout << "i2h:" << dv << endl;
		}
	}
	//set the updated weigths
	for(unsigned int i = 0; i < this->hidden.size(); ++i){
		this->hidden[i].w = w_ih[i];
	}
	for(unsigned int i = 0; i < this->outputs.size(); ++i){
		this->outputs[i].w = w_ho[i];
	}
}

double neural_network::train(vector<vector<double> > inputs, vector<vector<double> > expected){
	double mse;
	do{
		for(unsigned int i=0; i < inputs.size(); ++i){
			this->run(inputs[i]);
			this->calc_error(expected[i]);
			this->backpropagation();
			mse = this->calc_mse(expected[i]);
			//cout << mse << endl;
		}
	} while(mse > MSE);
	return mse;
	
}

double neural_network::calc_mse(vector<double> expected){
	double sum = 0;
	for(unsigned int i = 0; i < expected.size(); ++i){
		sum += pow(expected[i]-this->y2[i], 2);
	}
	return sum/2;
}
