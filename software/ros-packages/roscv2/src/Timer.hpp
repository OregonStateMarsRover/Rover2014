#ifndef TIMER_H
#define TIMER_H

#include <ros/ros.h>
#include <string>

class Timer {
public:
	Timer(std::string name) : Timer(name, 0) { }

	Timer(std::string name, int tabs) : name(name), tabs(tabs) {
		start = ros::Time::now();
#ifdef TIMER_OUTPUT
		disabled = false;
#else
		disabled = true;
#endif
	}

	~Timer() {
		if (!disabled) {
			ros::Duration duration = ros::Time::now() - start;
			std::string ts = "";
			for (int i = 0; i < tabs; i++) ts += "\t";
			ROS_INFO("%s%-16s\t%f", ts.c_str(), name.c_str(), duration.toSec());
		}
	}

	void disable() { disabled = true; }


private:
	ros::Time start;
	std::string name;
	int tabs;
	bool disabled;
};
#endif
