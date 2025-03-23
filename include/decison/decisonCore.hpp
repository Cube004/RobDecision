#ifndef DECISION_H
#define DECISION_H

#include "MoveBaseManager.hpp"
#include "roborts_msgs/driver.h"
#include "ostream"
#include <fstream>
#include <sstream>
#include <string>
#include "jsoncpp/json/json.h"
#include "rules/graph.hpp"
#include "database.hpp"


struct DecisionResult{
    std::vector<int> nodepath_;
    ros::Duration duration_;
    int target_id_;
};

struct DecisionConfig{
    std::string data_version_;
    std::string version_;
    std::string decisionJsonStr_;
    std::string decision_config_path_;
    std::string referee_topic_;
    std::string topic_cancel_goal_;
    std::string topic_send_goal_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool is_debug_;
};

class DecisionCore{
public:
    // 构造函数
    DecisionCore(DecisionConfig decision_config, ros::NodeHandle *nh):decision_config_(decision_config), nh_(nh){
        this->move_base_manager_ = std::make_unique<move_base_manager>
        (nh, decision_config_.frame_id_, decision_config_.child_frame_id_);
        this->init();
    }

    ~DecisionCore(){
        this->run_decision_ = false;
        this->run_subsriber_ = false;
        this->threads.join_all();
    }
    
    // 初始化函数，读取规则文件并排序
    void init(){
        //读取规则文件
        this->readRules();

        // 初始化订阅者
        this->threads.create_thread(boost::bind(&DecisionCore::initSubsriber, this));
    }

    // 读取规则文件的函数
    void readRules(){
        //读取规则文件
        std::ifstream file(decision_config_.decision_config_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + decision_config_.decision_config_path_);
        }
        Json::Value root;
        try {
            file >> root;
            this->decision_config_.data_version_ = root["Date"].asString();
            this->decision_config_.version_ = root["Version"].asString();
            
            this->decision_config_.decisionJsonStr_ = root.toStyledString();
            
            this->graph_.loadJson(this->decision_config_.decisionJsonStr_);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to parse JSON from file: " + decision_config_.decision_config_path_ + ". Error: " + e.what());
        }
        file.close();
    }

    
    // 运行函数
    void run(){
        // 决策
        this->run_decision_ = true;
        this->threads.create_thread(boost::bind(&DecisionCore::decision, this));
    }

    void stop(){
        this->run_decision_ = false;
    }

    void decision(){
        while(this->data_size_ < 10){
            ros::Rate(10).sleep();
        }
        while(this->run_decision_){
            this->decision_result_ = this->getTargetNode();
            #ifdef DEBUG
            ROS_INFO("target_node_id = %d, duration = %f, finish = %d", this->decision_result_.target_id_, this->decision_result_.duration_.toSec(), this->graph_.nodeList[this->decision_result_.target_id_]->get_finish());
            #endif
            for(auto node_id : this->decision_result_.nodepath_){
                std::cout << " " << node_id << " ";
            }
            std::cout << "target_node_id = " << this->decision_result_.target_id_ << std::endl;
            auto& target_node = this->graph_.nodeList[this->decision_result_.target_id_];
            this->database_.update_task_id(target_node->node_id);
            // 根据决策结果执行动作
            if (target_node->type == rules::NodeType::ROOT)
            {
                // 取消所有目标
                this->move_base_manager_->cancelAllGoal();
            }
            if(target_node->mode == rules::NodeMode::NAVIGATION){
                auto result = this->move_base_manager_->createWaypointTask(target_node->node_id, target_node->waypoint);
                target_node->start_time = result.task_start_time;
                target_node->end_time = result.task_completed_time;
                target_node->finish = result.completed;
            }else if(target_node->mode == rules::NodeMode::CHASE){
                // 追击, 暂时不实现
                this->move_base_manager_->cancelAllGoal();
            }else if(target_node->mode == rules::NodeMode::STAY){
                // 停留, 暂时不实现
                this->move_base_manager_->cancelAllGoal();
            }
            ros::Rate(10).sleep();
        }
    }

    DecisionResult getTargetNode(){
        DecisionResult decision_result;
        
        ros::Time start_time = ros::Time::now();

        auto node_id = this->graph_.rootNode_id;
        auto node = this->graph_.nodeList[node_id];
        auto edges = &node->edges;
        bool result = 1;

        while(this->graph_.CheckFinish(node_id) == true){// 尝试寻找任务节点，且只有完成的节点才能跳转到下一节点
            // 如果当前node_id已经在决策路径中，则执行该节点
            if(std::find(decision_result.nodepath_.begin(), decision_result.nodepath_.end(), node_id) != decision_result.nodepath_.end()){
                #ifdef DEBUG
                std::cout << "node_id: " << node_id << " in path" << std::endl;
                #endif
                decision_result.duration_ = ros::Time::now() - start_time;
                decision_result.target_id_ = node_id;
                return decision_result;
            }
            decision_result.nodepath_.push_back(node_id);
            #ifdef DEBUG
            std::cout << "edges size: " << edges->size() << std::endl;
            #endif
            for(auto edge : *edges){
                result = 1;
                #ifdef DEBUG
                std::cout << "edge_id: " << edge.edge_id << " condition size: " << edge.rules.conditions.size() << std::endl;
                #endif
                if(edge.rules.conditions.size() == 0) { // 如果当前节点没有跳转条件，则直接跳转到下一节点
                    node_id = edge.nodeOut_id;
                    node = this->graph_.nodeList[node_id];
                    edges = &node->edges;
                    break;
                }
                for(auto condition : edge.rules.conditions){
                    ros::Duration duration = ros::Duration(condition.scope_value);                    
                    if (condition.temporal_scope == rules::temporalScope::TASK_CYCLE){// 任务周期情况, 用于监控任务完成后一段时间的数值变化
                        if (node->get_finish() == true)
                        {
                            duration = ros::Time::now() - node->end_time;
                        }else{
                            break;
                        }
                    }
                    result = this->database_.check_data(condition.data_type, ros::Time::now(), 
                                    duration, 
                                    (database::metricType)condition.metric_type,  
                                    condition.min_value, 
                                    condition.max_value);
                    if(result == 0) break;// 如果有一个条件不满足，则跳出循环
                }
                if(result){// 如果所有条件都满足，则跳转到下一节点
                    node_id = edge.nodeOut_id;
                    node = this->graph_.nodeList[node_id];
                    edges = &node->edges;
                    break;
                }else if(edge.edge_id == edges->back().edge_id){// 如果已经遍历完所有节点，任然无法找到可跳转的下一节点，则返回当前节点
                    // 无法寻找到可跳转的下一节点，直接返回当前节点
                    decision_result.duration_ = ros::Time::now() - start_time;
                    decision_result.target_id_ = node_id;
                    return decision_result;
                }
            }
            if (ros::Time::now() - start_time > ros::Duration(0.5))
            {
                // 如果时间超过0.5秒，则返回当前节点, 可能有环
                decision_result.duration_ = ros::Time::now() - start_time;
                decision_result.target_id_ = node_id;
                return decision_result;
            }
        }
        // 寻找到未完成的任务节点
        decision_result.duration_ = ros::Time::now() - start_time;
        decision_result.target_id_ = node_id;
        return decision_result;
    }

    void initSubsriber(){
        this->run_subsriber_ = true;
        while(this->run_subsriber_){
            this->referee_sub_ = this->nh_->subscribe(decision_config_.referee_topic_, 10, &DecisionCore::referee_msgs_cb, this);
            ros::spinOnce();
        }
    }

    // 裁判系统消息回调函数
    void referee_msgs_cb(const roborts_msgs::driver::ConstPtr data){
        database_.update_data(*data);
        this->data_size_++;
    }


private:
    ros::NodeHandle *nh_;
    
    // 线程关闭信号
    std::atomic<bool> run_decision_;
    std::atomic<bool> run_subsriber_;

    std::atomic<int> data_size_;

    std::unique_ptr<move_base_manager> move_base_manager_;
    
    ros::Subscriber referee_sub_;

    DecisionConfig decision_config_;
    DecisionResult decision_result_;

    rules::Graph graph_;
    database::Database database_;

    boost::thread_group threads;
};





#endif // !DECISION_H