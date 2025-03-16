#ifndef MOVE_BASE_MANAGER_H
#define MOVE_BASE_MANAGER_H

#include "ros/ros.h"

#include <thread>
#include <chrono>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Empty.h>

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
    move_base_manager(ros::NodeHandle *nh, std::string topic_cancel_goal, std::string topic_send_goal, std::string frame_id, std::string child_frame_id): nh_(nh),running_(true){
        this->topic_cancel_goal_ = topic_cancel_goal;
        this->topic_send_goal_ = topic_send_goal;
        this->frame_id_ = frame_id;
        this->child_frame_id_ = child_frame_id;
        this->pub_cancel_goal_ = nh_->advertise<std_msgs::Empty>(topic_cancel_goal_, 10);
        this->pub_send_goal_ = nh_->advertise<geometry_msgs::PoseStamped>(topic_send_goal_, 10);
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
        pub_cancel_goal_.publish(std_msgs::Empty());
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
            if (abs(transform.transform.translation.x - goal.pose.position.x) > this->waypoint_task_.tolerance
            ||  abs(transform.transform.translation.y - goal.pose.position.y) > this->waypoint_task_.tolerance)
            {
                pub_send_goal_.publish(goal);
            }
            else{
                this->waypoint_task_.task_completed_time = ros::Time::now();
                this->waypoint_task_.completed = true;
            }
        }
        return this->waypoint_task_;
    }

private:

    void run(){
        loop_thread = std::make_unique<std::thread>(&move_base_manager::loop, this); // 启动线程
    }

    void loop(){
        while(this->running_ && ros::ok()){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                try{
                    transform = tf_buffer->lookupTransform(frame_id_, child_frame_id_, ros::Time(0));
                }catch(tf2::TransformException &ex){
                    ROS_WARN("%s", ex.what());
                }
            }
            ros::Duration(0.04).sleep();
        }
    }

private:
    WaypointTask waypoint_task_;
    std::atomic<bool> running_;
    std::mutex mutex_; // 用于保护共享资源的互斥锁
    ros::NodeHandle *nh_;
    
    std::unique_ptr<std::thread> loop_thread;

    ros::Publisher pub_cancel_goal_;
    ros::Publisher pub_send_goal_;
    
    std::string topic_cancel_goal_;
    std::string topic_send_goal_;

    std::string frame_id_;
    std::string child_frame_id_;

    // 用于获取机器人位置
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::TransformStamped transform;
};

#endif // MOVE_BASE_MANAGER_H