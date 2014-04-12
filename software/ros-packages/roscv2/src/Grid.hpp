#include <roscv2/Grid.h>

class Grid {
private:
	int **ptr;
	int _width, _height;
	float _real_width, _real_height;

	void init() {
		for (int i = 0; i < _width; i++) {
			for (int j = 0; j < _height; j++) {
				ptr[i][j] = 0;
			}
		}
	}
public:
	Grid(int w, int h, float w_m, float h_m) {
		_width = w; _height = h;
		_real_width = w_m; _real_height = h_m;
		ptr = new int*[_width];
		for (int i = 0; i < _width; i++) ptr[i] = new int[_height];
		init();
	}

	~Grid() {
		for (int i = 0; i < _width; i++) delete[] ptr[i];
		delete ptr;
	}

	int width() const { return _width; }
	int height() const { return _height; }
	int real_width() const { return _real_width; }
	int real_height() const { return _real_height; }

	int const * operator[](int i) const {
		return (int const *)ptr[i];
	}
	int * operator[](int i) {
		return ptr[i];
	}
	void populate_msg(roscv2::Grid &msg) const {
		msg.width = _width;
		msg.height = _height;
		msg.width_m = _real_width;
		msg.height_m = _real_width;
		for (int j = 0; j < _height; j++) {
			for (int i = 0; i < _width; i++) {
				msg.data.push_back(ptr[i][j]);
			}
		}
	}
	static Grid from_msg(const roscv2::Grid &msg) {
		int width = msg.width;
		int height = msg.height;
		float real_width = msg.width_m;
		float real_height = msg.height_m;
		Grid g = Grid(width, height, real_width, real_height);
		for (int j = 0; j < height; j++) {
			for (int i = 0; i < width; i++) {
				g[i][j] = msg.data[j*width+i];
			}
		}
		return g;
	}
};
