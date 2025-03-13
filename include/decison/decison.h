#ifndef DECISION_H
#define DECISION_H

#include "move_base_manager.h"
#include "roborts_msgs/driver.h"
#include "ostream"

#include "rules/graph.h"

class Decision{
public:
    // 构造函数
    Decision();
    // 析构函数
    ~Decision();
    // 初始化函数，读取规则文件并排序
    void init(){
        //读取规则文件
        read_rules();

    }

    // 读取规则文件的函数
    void read_rules(){
        //读取规则文件
    }
    
    // 运行函数
    void run(){

    }

    // 参考消息回调函数
    void referee_msgs_cb(const roborts_msgs::driver::ConstPtr data){
        // 根据规则进行决策
        auto node = graph_.into_graph(data);
        
    }


private:
    move_base_manager* move_base_manager_;
    ros::NodeHandle nh_;
    ros::Subscriber referee_sub_;
    Graph graph_;
};





#endif // !DECISION_H