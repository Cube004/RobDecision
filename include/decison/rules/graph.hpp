#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "rules.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "jsoncpp/json/json.h"

namespace decision {
    enum NodeType {
        ROOT,
        TASK
    };

    enum NodeMode {
        WAYPOINT,
        CHASE,
        STAY
    };

    struct Edge {
        int edge_id;
        int nodeIn_id;
        int nodeOut_id;
        int weight;
        rules::DecisionRules rules;
    };

    struct Node {
        int node_id;
        NodeType type;
        NodeMode mode;
        geometry_msgs::PoseStamped waypoint;
        float resetTime;
        
        bool finish;
        ros::Time last_time;// 上一次到达该节点的时间

        bool GetState(){
            if (resetTime == -1) return finish;
            // 非一次性任务超过指定时间未到达该节点认为是冷数据, 一段时间后重置完成状态
            // (未到达该节点也可理解为任务决策在上一段时间偏离转而执行其他路线上的节点)
            if (finish && ros::Time::now() - last_time > ros::Duration(resetTime)) {
                finish = false;
                return false;
            }else {
                return finish;
            }
        }
        std::vector<Edge> edges;
    };



class Graph {
    public:
        std::unordered_map<int, Node> nodeList;
        std::unordered_map<int, Edge> edgeList;
        std::string decisionJsonStr;
        int rootNode_id = -1;
        // Graph();
        bool init(const std::string& decisionJsonStr);
        void AddNode(Json::Value node);
        void AddEdge(Json::Value edge);
        void loadJson(std::string decisionJsonStr);
        void AddRule(Json::Value rule, rules::DecisionRules &rules);
        std::vector<int> nextNode(roborts_msgs::driver *data);
        int matchEdge(roborts_msgs::driver *data, std::vector<Edge> *edges);
        std::string ParseJsonToString(const Json::Value& jsonValue);
    };

    // 初始化图
    bool Graph::init(const std::string& decisionJsonStr){
        try
        {
            this->decisionJsonStr = decisionJsonStr;
            this->nodeList.clear();
            this->edgeList.clear();
            this->loadJson(this->decisionJsonStr);
        }
        catch(const std::exception& e)
        {
            std::cerr << "规则加载失败: " << e.what() << '\n';
            return false;
        }
        return true;
    }

    // 加载json文件
    void Graph::loadJson(std::string decisionJsonStr){
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccessful = reader.parse(decisionJsonStr, root);
        if (!parsingSuccessful) {
            throw std::runtime_error("Failed to parse JSON from string: " + reader.getFormattedErrorMessages());
        }
        
        Json::Value nodes = root["Nodes"];
        Json::Value edges = root["Edges"];
        // 添加节点
        for (auto& node : nodes) {
            std::cout << "node: " << node << std::endl;
            this->AddNode(node);
        }
        // 添加条件跳转边
        for (auto& edge : edges) {
            this->AddEdge(edge);
        }
        // 将边添加到节点中
        for (auto& edge : edgeList) {
            this->nodeList[edge.second.nodeIn_id].edges.push_back(edge.second);
        }
        // 将节点中的条件跳转边按权重排序，权重越大优先级越高
        for (auto& node : nodeList) {
            std::sort(node.second.edges.begin(), node.second.edges.end(), [](const Edge& a, const Edge& b) {
                return a.weight > b.weight;
            });
        }
        if (this->rootNode_id == -1) {
            throw std::runtime_error("rootNode_id is not set");
        }

        for (auto& node : nodeList) {
            std::cout << "node_id: " << node.second.node_id << " edgesNumber: " << node.second.edges.size() << std::endl;
        }
        for (auto& edge : edgeList) {
            std::cout << "edge_id: " << edge.second.edge_id << " nodeIn_id: " << edge.second.nodeIn_id << " nodeOut_id: " << edge.second.nodeOut_id << " weight: " << edge.second.weight << std::endl;
            for (const auto& condition : edge.second.rules.conditions) {
                if (condition.match_type == rules::MatchType::IN_RANGE)
                {
                    std::cout << "condition_type: " << condition.type << " min_value: " << condition.min_value << " max_value: " << condition.max_value << " match_type: " << condition.match_type << std::endl;
                }
            }
        }
    }


    // 添加节点
    void Graph::AddNode(Json::Value node){
        Node newNode;
        std::string node_id = node["id"].asString();
        node_id.erase(0, 5);
        newNode.node_id = std::stoi(node_id);
        std::cout << "node_id: " << node_id << std::endl;
        if (node["taskConfig"]["nodeType"].asString() == "task") {
            newNode.type = NodeType::TASK;
        } else if (node["taskConfig"]["nodeType"].asString() == "root") {
            newNode.type = NodeType::ROOT;
            this->rootNode_id = newNode.node_id;
        }

        if (node["taskConfig"]["mode"].asString() == "waypoint") {
            newNode.mode = NodeMode::WAYPOINT;
        } else if (node["taskConfig"]["mode"].asString() == "chase") {
            newNode.mode = NodeMode::CHASE;
        } else if (node["taskConfig"]["mode"].asString() == "stay") {
            newNode.mode = NodeMode::STAY;
        }

        std::string waypointStr = this->ParseJsonToString(node["taskConfig"]["waypoint"]);
        if (waypointStr != "null")
        {
            Json::Value waypointValue;
            Json::CharReaderBuilder reader;
            std::string errs;
            std::istringstream sstream(node["taskConfig"]["waypoint"].asString());

            if (!Json::parseFromStream(reader, sstream, &waypointValue, &errs)) {
                throw std::runtime_error("Failed to parse waypoint JSON: " + errs);
            }

            // 访问解析后的 waypoint 对象
            std::cout << "waypointValue: " << waypointValue << std::endl;
            newNode.waypoint.pose.position.x = waypointValue["x"].asFloat();
            newNode.waypoint.pose.position.y = waypointValue["y"].asFloat();
        }

        newNode.last_time = ros::Time::now();


        std::string resetTimeStr = this->ParseJsonToString(node["taskConfig"]["resetTime"]);

        if (resetTimeStr != "null")
        {
            std::cout << "resetTimeStr: " << resetTimeStr << std::endl;
            newNode.resetTime = std::stof(resetTimeStr);
        }else{
            newNode.resetTime = -1;
        }
        nodeList[newNode.node_id] = newNode;
    }

    void Graph::AddEdge(Json::Value edge){
        Edge newEdge;
        std::cout << "edge: " << edge << std::endl;
        std::string edge_id = edge["id"].asString();
        std::string nodeIn = edge["condition"]["nodeId"]["nodeIn"].asString();
        std::string nodeOut = edge["condition"]["nodeId"]["nodeOut"].asString();
        std::cout << "edge_id: " << edge_id << " nodeIn: " << nodeIn << " nodeOut: " << nodeOut << std::endl;
        edge_id.erase(0, 5);
        nodeIn.erase(0, 5);
        nodeOut.erase(0, 5);
        std::cout << "edge_id: " << edge_id << " nodeIn: " << nodeIn << " nodeOut: " << nodeOut << std::endl;
        newEdge.edge_id = std::stoi(edge_id);
        newEdge.nodeIn_id = std::stoi(nodeIn);
        newEdge.nodeOut_id = std::stoi(nodeOut);

        std::cout << "weight: " << edge["condition"]["weight"].asString() << std::endl;
        newEdge.weight = edge["condition"]["weight"].asInt();
        for (const auto& condition : edge["condition"]["condition"]) {
            this->AddRule(condition, newEdge.rules);
        }
        edgeList[newEdge.edge_id] = newEdge;
    }

    void Graph::AddRule(Json::Value condition, rules::DecisionRules &rules){
        for (size_t i = 0; i < rules.conditions.size(); i++)
        {
            if (condition["type"].asString() == rules.conditions[i].type)
            {
                rules.conditions[i].max_value = condition["max"].asFloat();
                rules.conditions[i].min_value = condition["min"].asFloat();
                rules.conditions[i].match_type = rules::MatchType::IN_RANGE;
                rules.condition_count++;
                break;
            }
        }
    }

    // 获取下一个节点
    std::vector<int> Graph::nextNode(roborts_msgs::driver *data){
        std::vector<int> targetNode_path;
        int targetNode_id = -1;
        targetNode_id = matchEdge(data, &this->nodeList[this->rootNode_id].edges);
        while (this->matchEdge(data, &this->nodeList[targetNode_id].edges) != -1 && this->nodeList[targetNode_id].GetState() == false) {
            targetNode_path.push_back(targetNode_id);
            targetNode_id = matchEdge(data, &this->nodeList[targetNode_id].edges);
        }
        return targetNode_path;
    }

    // 匹配边
    int Graph::matchEdge(roborts_msgs::driver *data, std::vector<Edge> *edges){
        for (const auto& edge : *edges) {
            if (rules::match_rules(edge.rules, data)) {
                return edge.nodeOut_id;
            }
        }
        return -1;
    }

    std::string Graph::ParseJsonToString(const Json::Value& jsonValue) 
    {
        Json::StreamWriterBuilder writerBuilder;
        writerBuilder["indentation"] = ""; // 紧凑格式
        writerBuilder.settings_["precision"] = 6; // 设置浮点数精度
        writerBuilder.settings_["precisionType"] = "decimal"; // 十进制格式

        try {
            // 使用更安全的工厂方法创建 writer
            if (auto writer = writerBuilder.newStreamWriter()) {
                std::ostringstream oss;
                oss.exceptions(std::ios_base::badbit | std::ios_base::failbit);
                
                // 验证 JSON 有效性
                if (jsonValue.isNull()) {
                    return "null";  // 明确处理 null 值
                }
                
                writer->write(jsonValue, &oss);
                return oss.str();
            }
        } 
        catch (const Json::Exception& e) {
            // 精准捕获 JSON 相关异常
        }
        catch (const std::ios_base::failure& e) {
            // 处理流错误
        }
        catch (...) {
            // 捕获所有其他异常
        }

        return "null";  // 统一异常出口
    }
}

#endif // GRAPH_H
