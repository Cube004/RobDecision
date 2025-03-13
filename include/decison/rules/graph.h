#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "rules.h"
#include "geometry_msgs/PoseStamped.h"

struct Node {
    int id;
    geometry_msgs::PoseStamped goal;
    bool finish;
    bool disposable;
    ros::Time last_time;// 上一次到达该节点的时间
    ros::Duration timeout;

    bool getstate(){
        if (disposable) return finish;

        // 非一次性任务超过指定时间未到达该节点
        // (未到达该节点也可理解为任务决策在上一段时间偏离转而执行其他路线上的节点)
        // 则重置完成状态
        if (finish && ros::Time::now() - last_time > timeout) {
            finish = false;
            return false;
        }else {
            return finish;
        }
    }

    void setstate(bool state){
        finish = state;
    }
};

struct Edge {
    int from;
    int to;
    double weight;
    rules::decison_rules rule;
};

class Graph {
private:
    std::unordered_map<int, Node> nodes;
    std::unordered_map<int,std::vector<Edge>> edges_lists;

public:
    Graph();
    void init();
    Node* into_graph(roborts_msgs::driver::ConstPtr data);
    Node* next(Node* node,roborts_msgs::driver::ConstPtr data);
private:
    void add_node(Node node);
    void add_edge(Edge edge);

};

Node* Graph::into_graph(roborts_msgs::driver::ConstPtr data) {
    Node* node = &nodes[0];
    Node* next_node = nullptr;
    // 当目前节点是已完成状态，就试图跳转到下一个
    // 如果跳转失败或者发现当前节点未完成，就停
    while (node->getstate()) {
        Node* candidate = next(node, data);
        if (!candidate) break; // 跳转失败
        node = candidate;
    }
    return node;
}

Node* Graph::next(Node* node,roborts_msgs::driver::ConstPtr data) {
    for (size_t i = 0; i < this->edges_lists[node->id].size(); i++){
        auto next_node = this->edges_lists[node->id][i];
        if(rules::match_rules(next_node.rule,data)){
            nodes[next_node.to].last_time = ros::Time::now();
            return &this->nodes[next_node.to];
        }
    }
    return nullptr;
}

void Graph::add_node(Node node) {
    if (nodes.count(node.id)) {
            std::cerr << "节点 " << node.id << " 已存在！" << std::endl;
            return;
    }
    nodes[node.id] = node;
}

void Graph::add_edge(Edge edge) {
    if (!nodes.count(edge.from) || !nodes.count(edge.to)){
        std::cerr << "节点 " << edge.from << " 或 " << edge.to << " 不存在！" << std::endl;
        return;
    }

    if (edges_lists.count(edge.from)) {
        edges_lists[edge.from].push_back(edge);
    } else {
        edges_lists[edge.from] = std::vector<Edge>();
        edges_lists[edge.from].push_back(edge);
    }
}

#endif // GRAPH_H