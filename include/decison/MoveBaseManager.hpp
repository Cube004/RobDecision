#ifndef MOVE_BASE_MANAGER_H
#define MOVE_BASE_MANAGER_H

#include "ros/ros.h"

#include <thread>
#include <chrono>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"


struct WaypointTask 
{
    int32_t task_id = -1;
    ros::Time task_start_time;
    ros::Time task_completed_time;
    geometry_msgs::PoseStamped waypoint_pose;
    float tolerance = 0.3;
    bool completed = false;
};


class move_base_manager{

public:
    move_base_manager(ros::NodeHandle *nh, std::string frame_id, std::string child_frame_id): nh_(nh),running_(true){
        this->client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(*nh_, "move_base", true);
        this->frame_id_ = frame_id;
        this->child_frame_id_ = child_frame_id;
        this->tf_buffer = std::make_unique<tf2_ros::Buffer>();
        this->tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
        this->run();
    }
    ~move_base_manager(){
        running_ = false;
        if (loop_thread && loop_thread->joinable()) {
            loop_thread->join();
        }
        printf("move_base_manager is destoryed\n");
    }

    void cancelAllGoal(){
        this->client_->cancelAllGoals();
        this->waypoint_task_.task_id = -1;
    }


    WaypointTask createWaypointTask(int id, geometry_msgs::PoseStamped goal){
        if (this->waypoint_task_.task_id != id){ // 新任务
            this->waypoint_task_.task_id = id;
            this->waypoint_task_.task_start_time = ros::Time::now();
            this->waypoint_task_.task_completed_time = ros::Time(0);
            this->waypoint_task_.waypoint_pose = goal;
            this->waypoint_task_.completed = false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (abs(transform.transform.translation.x - goal.pose.position.x) > this->waypoint_task_.tolerance || 
                abs(transform.transform.translation.y - goal.pose.position.y) > this->waypoint_task_.tolerance)
            {   // 如果当前位置与目标位置的距离大于阈值, 且当前目标点与目标点不一致, 则发送目标点
                if ((abs(current_goal.pose.position.x - goal.pose.position.x) > 0.01 || abs(current_goal.pose.position.y - goal.pose.position.y) > 0.01))
                {  
                    move_base_msgs::MoveBaseGoal goal_msg;
                    goal_msg.target_pose.pose.position.x = goal.pose.position.x;
                    goal_msg.target_pose.pose.position.y = goal.pose.position.y;
                    goal_msg.target_pose.pose.position.z = 0;
                    goal_msg.target_pose.pose.orientation.w = 1;
                    goal_msg.target_pose.pose.orientation.x = 0;
                    goal_msg.target_pose.pose.orientation.y = 0;
                    goal_msg.target_pose.pose.orientation.z = 0;
                    goal_msg.target_pose.header.frame_id = "map";
                    goal_msg.target_pose.header.stamp = ros::Time::now();
                    client_->sendGoal(goal_msg);
                }
                #ifdef DEBUG
                std::cout << "send goal" << "id: " << id << "  x: " << goal.pose.position.x << "y: " << goal.pose.position.y << std::endl;
                #endif
            }
            else{
                #ifdef DEBUG
                std::cout << "task completed" << "id: " << id << "  x: " << transform.transform.translation.x << "y: " << transform.transform.translation.y << std::endl;
                #endif
                this->waypoint_task_.task_completed_time = ros::Time::now();
                this->waypoint_task_.completed = true;
            }
        }
        return this->waypoint_task_;
    }

    std::pair<float, float> getRobotPosition(){
        std::lock_guard<std::mutex> lock(mutex_);
        return std::make_pair(transform.transform.translation.x, transform.transform.translation.y);
    }

private:

    void run(){
        loop_thread = std::make_unique<std::thread>(&move_base_manager::loop, this); // 启动线程
    }

    void loop(){
        sub_current_goal = std::make_unique<ros::Subscriber>(nh_->subscribe("/move_base/current_goal", 1, &move_base_manager::current_goal_callback, this));
        while(this->running_ && ros::ok()){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                try{
                    transform = tf_buffer->lookupTransform(frame_id_, child_frame_id_, ros::Time(0));
                }catch(tf2::TransformException &ex){
                    ROS_WARN("%s", ex.what());
                }
            }
            #ifdef DEBUG
            std::cout << "transform x: " << transform.transform.translation.x << " y: " << transform.transform.translation.y << std::endl;
            #endif
            ros::Duration(0.04).sleep();
            ros::spinOnce();
        }
    }

    void current_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_goal = *msg;
        // #ifdef DEBUG
        std::cout << "current_goal x: " << current_goal.pose.position.x << " y: " << current_goal.pose.position.y << std::endl;
        // #endif
    }

private:
    WaypointTask waypoint_task_;
    std::atomic<bool> running_;
    std::mutex mutex_; // 用于保护共享资源的互斥锁
    ros::NodeHandle *nh_;

    std::unique_ptr<ros::Subscriber> sub_current_goal;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> client_;
    std::unique_ptr<std::thread> loop_thread;


    std::string frame_id_;
    std::string child_frame_id_;

    // 用于获取机器人位置
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::TransformStamped transform;

    // 用于获取当前目标点
    geometry_msgs::PoseStamped current_goal;
};

#endif // MOVE_BASE_MANAGER_H