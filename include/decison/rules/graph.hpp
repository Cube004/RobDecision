#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "rules.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "jsoncpp/json/json.h"

namespace rules {
    enum NodeType {
        ROOT,
        TASK
    };

    enum NodeMode {
        NAVIGATION,
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
        ros::Time last_check_time;// 上一次检查该节点的时间
        ros::Time start_time;
        ros::Time end_time;
        bool get_finish(){
            if (this->type == NodeType::ROOT || this->mode == NodeMode::CHASE || this->mode == NodeMode::STAY){
                this->finish = true;
                return this->finish;
            }
            
            if (resetTime == -1) return finish;
            // 非一次性任务超过指定时间未到达该节点认为是冷数据, 一段时间后重置完成状态
            // (未到达该节点也可理解为任务决策在上一段时间偏离转而执行其他路线上的节点)
            if (finish && ros::Time::now() - last_check_time > ros::Duration(resetTime)) {
                this->end_time = ros::Time(0);
                this->finish = false;
                return false;
            }else {
                return finish;
            }
            last_check_time = ros::Time::now();// 更新检查时间
        }
        std::vector<Edge> edges;
    };



class Graph {
    public:
        std::unordered_map<int, Node> nodeList;
        std::unordered_map<int, Edge> edgeList;
        std::string decisionJsonStr;
        int rootNode_id = -1;
        bool init(const std::string& decisionJsonStr);
        void AddNode(Json::Value node);
        void AddEdge(Json::Value edge);
        void loadJson(std::string decisionJsonStr);
        void AddRule(Json::Value rule, rules::DecisionRules &rules);
        std::string ParseJsonToString(const Json::Value& jsonValue, std::string value = "null");
        void Debuginfo();
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
        this->nodeList.clear();
        this->edgeList.clear();
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
        this->Debuginfo();
    }


    // 添加节点
    void Graph::AddNode(Json::Value node){
        Node newNode;
        std::string node_id = node["id"].asString();
        node_id.erase(0, 5);
        newNode.node_id = std::stoi(node_id);
        
        // 节点类型
        if (node["taskConfig"]["nodeType"].asString() == "task") {
            newNode.type = NodeType::TASK;
        } else if (node["taskConfig"]["nodeType"].asString() == "root") {
            newNode.type = NodeType::ROOT;
            this->rootNode_id = newNode.node_id;
        }
        // 节点模式
        if (node["taskConfig"]["mode"].asString() == "navigation") {
            newNode.mode = NodeMode::NAVIGATION;
        } else if (node["taskConfig"]["mode"].asString() == "chase") {
            newNode.mode = NodeMode::CHASE;
        } else if (node["taskConfig"]["mode"].asString() == "stay") {
            newNode.mode = NodeMode::STAY;
        }

        // 航点坐标
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
            newNode.waypoint.pose.position.x = waypointValue["x"].asFloat();
            newNode.waypoint.pose.position.y = waypointValue["y"].asFloat();
        }else{
            newNode.waypoint.pose.position.x = -1;
            newNode.waypoint.pose.position.y = -1;
        }

        newNode.last_check_time = ros::Time::now();

        newNode.resetTime = std::stoi(this->ParseJsonToString(node["taskConfig"]["resetTime"], "-1"));
        nodeList[newNode.node_id] = newNode;
    }

    void Graph::AddEdge(Json::Value edge){
        Edge newEdge;
        std::string edge_id = edge["id"].asString();
        std::string nodeIn = edge["condition"]["nodeId"]["nodeIn"].asString();
        std::string nodeOut = edge["condition"]["nodeId"]["nodeOut"].asString();
        edge_id.erase(0, 5);
        nodeIn.erase(0, 5);
        nodeOut.erase(0, 5);
        newEdge.edge_id = std::stoi(edge_id);
        newEdge.nodeIn_id = std::stoi(nodeIn);
        newEdge.nodeOut_id = std::stoi(nodeOut);

        newEdge.weight = edge["condition"]["weight"].asInt();
        for (auto& condition : edge["condition"]["condition"]) {
            this->AddRule(condition, newEdge.rules);
        }
        edgeList[newEdge.edge_id] = newEdge;
    }

    void Graph::AddRule(Json::Value condition, rules::DecisionRules &rules){
        int max_value = condition["max"].asInt();
        int min_value = condition["min"].asInt();
        rules::MatchType match_type = rules::MatchType::IN_RANGE;

        std::string type = condition["datetype"].asString();
        int metric_type = condition["metricType"].asInt();

        int temporal_scope = stoi(this->ParseJsonToString(condition["temporalScope"]["type"], "0"));
        int scope_value = stoi(this->ParseJsonToString(condition["temporalScope"]["rollingWindow"], "0"));
        
        rules::RuleCondition rule_condition(type);
        rule_condition.change_value(min_value, max_value, match_type, metric_type, temporal_scope, scope_value);
        rules.add_condition(rule_condition);
    }

    std::string Graph::ParseJsonToString(const Json::Value& jsonValue, std::string value) 
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
                    return value != "null" ? value : "null";  // 明确处理 null 值
                }
                
                writer->write(jsonValue, &oss);
                std::string result = oss.str();

                // 如果 jsonValue 是字符串类型，去除两侧的双引号
                if (jsonValue.isString()) {
                    if (result.size() >= 2 && result.front() == '"' && result.back() == '"') {
                        result = result.substr(1, result.size() - 2);
                    }
                }

                return result;
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

        return value;  // 统一异常出口
    }

    void Graph::Debuginfo(){
        for (const auto& node : this->nodeList) {
            std::cout << "----------------START---------------" << std::endl;
            std::cout << "node_id: " << node.second.node_id << std::endl;
            std::cout << "type: " << node.second.type << std::endl;
            std::cout << "mode: " << node.second.mode << std::endl;
            std::cout << "waypoint: " << node.second.waypoint.pose.position.x << ", " << node.second.waypoint.pose.position.y << std::endl;
            std::cout << "resetTime: " << node.second.resetTime << std::endl;
            for (const auto& edge : node.second.edges) {
                std::cout << "--------------------------------" << std::endl;
                std::cout << "  edge_id: " << edge.edge_id << std::endl;
                std::cout << "  nodeIn_id: " << edge.nodeIn_id << std::endl;
                std::cout << "  nodeOut_id: " << edge.nodeOut_id << std::endl;
                std::cout << "  weight: " << edge.weight << std::endl;
                for (const auto& rule : edge.rules.conditions) {
                    std::cout << "--------------------------------" << std::endl;
                    std::cout << "      type: " << rule.datatype << std::endl;
                    std::cout << "      min_value: " << rule.min_value << std::endl;
                    std::cout << "      max_value: " << rule.max_value << std::endl;
                    std::cout << "      match_type: " << rule.match_type << std::endl;
                    std::cout << "      metric_type: " << rule.metric_type << std::endl;
                    std::cout << "      temporal_scope: " << rule.temporal_scope << std::endl;
                    std::cout << "      scope_value: " << rule.scope_value << std::endl;
                }
            }
            std::cout << "----------------END-----------------" << std::endl;
        }
    }
}

#endif // GRAPH_H
