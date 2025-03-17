#include "ros/ros.h"
#include "decison/decisonCore.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "decison");
    ros::NodeHandle n;
    n.setParam("/use_sim_time", true);
    DecisionConfig decision_config;
    decision_config.decision_config_path_ = "/home/cube/Documents/decision/output.json";
    decision_config.referee_topic_ = "/referee/msg";
    decision_config.frame_id_ = "odom";
    decision_config.child_frame_id_ = "base_footprint";

    DecisionCore decision(decision_config, &n);

    decision.run();

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
