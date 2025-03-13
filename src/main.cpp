#include "ros/ros.h"
#include "decison/move_base_manager.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "decison");
    ros::NodeHandle n;
    move_base_manager manager(&n);
    
    std::thread user_input_thread([&]() {
        while (ros::ok()) {
            int id;
            std::cin >> id;  // 推荐使用 std::cin
            if (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                ROS_WARN("Invalid input. Please enter a number.");
                continue;
            }
            ROS_INFO("User input: %d", id);
        }
    });

    while (ros::ok()) {
        ros::spinOnce();
    }

    user_input_thread.join();  // 等待输入线程结束
    return 0;
}
