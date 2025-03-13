#ifndef DECISION_H
#define DECISION_H

#include "move_base_manager.h"
#include "roborts_msgs/driver.h"
#include "ostream"
#include <fstream>
#include <sstream>
#include <string>
#include "jsoncpp/json/json.h"
#include "rules/graph.hpp"

class Decision{
public:
    // 构造函数
    Decision(std::string decision_config_path):decision_config_path_(decision_config_path){
        this->init();
    }

    // 初始化函数，读取规则文件并排序
    void init(){
        //读取规则文件
        read_rules();


    }

    // 读取规则文件的函数
    void read_rules(){
        //读取规则文件
        std::ifstream file(decision_config_path_);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + decision_config_path_);
        }
        Json::Value root;
        try {
            file >> root;
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to parse JSON from file: " + decision_config_path_ + ". Error: " + e.what());
        }

        this->data_version_ = root["Date"].asString();
        this->version_ = root["Version"].asString();
        
        std::string decisionJsonStr = root.toStyledString();
        
        this->graph_.loadJson(decisionJsonStr);
        file.close();
    }

    
    // 运行函数
    void run(){
        // 决策
        this->nodepath_ = graph_.nextNode(&referee_msgs_buffer_.back());
    }

    // 参考消息回调函数
    void referee_msgs_cb(const roborts_msgs::driver::ConstPtr data){
        referee_msgs_buffer_.push_back(*data);
    }


private:
    move_base_manager* move_base_manager_;
    ros::NodeHandle nh_;
    ros::Subscriber referee_sub_;
    std::vector<roborts_msgs::driver> referee_msgs_buffer_;
    decision::Graph graph_;
    std::vector<int> nodepath_;
    std::string decision_config_path_;
    std::string data_version_;
    std::string version_;
};





#endif // !DECISION_H