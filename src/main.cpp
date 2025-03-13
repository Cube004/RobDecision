#include "ros/ros.h"
#include "decison/move_base_manager.h"
#include "decison/decison.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "decison");
    ros::NodeHandle n;
    move_base_manager manager(&n);
    
    Decision decision(std::string("/home/cube/Documents/decision/output (11).json"));

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
