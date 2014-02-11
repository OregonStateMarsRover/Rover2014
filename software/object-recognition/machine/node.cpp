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
	int counter = 0;
	vector<int> v;
	for(int i=0; i < MAXCLASS; ++i){
		v.push_back(classification == i);
	}
	return v;
}

int main(){
	cout << "test" << endl;
	node n(2, 3, sqrt(3));
	cout << n.get_w1() << endl;
	return 0;
}
