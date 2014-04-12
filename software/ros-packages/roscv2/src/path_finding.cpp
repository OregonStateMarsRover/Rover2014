#include "obstacle_detect.hpp"
#include <roscv2/grid.h>

void gridCallback(const roscv2::Grid& msg);

void gridCallback(const roscv2:Grid& msg) {
    int grid[msg.x][msg.y];
    int i,j,k;

    for (i=0;i<x;i++) {
        for (j=0;j<y;j++){
            grid[i][k] = msg.array[k];
            k++;
        }
    }

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "path finding");
    ros::NodeHandle pf;
    ROS_INFO("Path finding node online");

    ros::Subscriber sub = pf.subscribe("obstacle_grid", 10, gridCallback);

    ros::SpinOnce;
}
