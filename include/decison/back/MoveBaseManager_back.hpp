#ifndef MOVE_BASE_MANAGER_H
#define MOVE_BASE_MANAGER_H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

#include <thread>
#include <chrono>

struct goal_task
{
    int id = -1;
    ros::Time start_time;
    ros::Time finish_time;
    geometry_msgs::PoseStamped goal;
};

struct log_info
{
    goal_task task;
    ros::Duration duration_time;
};

class move_base_manager{
public:
    move_base_manager(ros::NodeHandle *nh): nh_(nh),running_(true),callback(nullptr){
        this->run();
    }
    ~move_base_manager(){
        running_ = false;
        if (loop_thread && loop_thread->joinable()) {
            loop_thread->join();
        }
        printf("move_base_manager is destoryed\n");
    }

    void cancel_goal(){
        std::lock_guard<std::mutex> lock(mutex_);
        if (task.id != -1){
            ros::Duration duration_time;
            if(state->isDone()){
                duration_time = ros::Time::now() - task.finish_time;
            }else{
                duration_time = ros::Duration(0);
            }
            log_list.push_back({task, duration_time});
            client_->cancelAllGoals();
            task.id = -1;
        }
    }

    void send_goal(int id, geometry_msgs::PoseStamped goal){
        std::lock_guard<std::mutex> lock(mutex_);
        if( task.id == id) return;

        if (task.id != -1){
            ros::Duration duration_time;
            if(state->isDone()){
                duration_time = ros::Time::now() - task.finish_time;
            }else{
                duration_time = ros::Duration(0);
            }
            log_list.push_back({task, duration_time});
        }

        task = {id, ros::Time::now(), ros::Time(0), goal};

        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose = goal;
        client_->sendGoal(goal_msg);
    }

    actionlib::SimpleClientGoalState get_state(){
        std::lock_guard<std::mutex> lock(mutex_);
        return *state;
    }

    std::vector<log_info> get_log(){
        std::lock_guard<std::mutex> lock(mutex_);
        return log_list;
    }

    void bind_done_callback(std::function<void(goal_task)> callback){
        std::lock_guard<std::mutex> lock(mutex_);
        this->callback = callback;
    }
private:

    void run(){
        this->state = std::make_unique<actionlib::SimpleClientGoalState>(actionlib::SimpleClientGoalState::LOST); // 初始化状态
        this->client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(*nh_, "move_base", true);
        loop_thread = std::make_unique<std::thread>(&move_base_manager::loop, this); // 启动线程
    }

    void loop(){
        client_->waitForServer();// 等待服务器启动
        printf("move_base_manager is loopping\n");
        while(this->running_ && ros::ok()){
            // 获取当前状态
            {
                std::lock_guard<std::mutex> lock(mutex_);
                *state = client_->getState();
            
                // 检查任务是否完成
                if (state->isDone() && task.id != -1 && 
                    ros::Time::now() - task.start_time > ros::Duration(1))
                {
                    task.finish_time = ros::Time::now();
                    if(callback != nullptr) callback(task);
                }
            }
            ros::Duration(0.04).sleep();
        }
        printf("move_base_manager is stoped\n");
    }

private:
    std::atomic<bool> running_;
    std::mutex mutex_; // 用于保护共享资源的互斥锁
    ros::NodeHandle *nh_;
    std::unique_ptr<std::thread> loop_thread;
    std::function<void(goal_task)> callback;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> client_;
    std::unique_ptr<actionlib::SimpleClientGoalState> state;
    goal_task task;
    std::vector<log_info> log_list;
};

#endif // MOVE_BASE_MANAGER_H