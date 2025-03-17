#ifndef RULES_H
#define RULES_H

#include <iostream>
#include <vector>
#include <functional>
#include "roborts_msgs/driver.h"
#include "decison/database.hpp"

namespace rules {
    // 匹配类型枚举
    enum MatchType {
        IN_RANGE,         // 在范围内
        IGNORE            // 忽略此条件
    };

    enum metricType{
        CURRENT_VALUE = 1,       // 当前数值
        TOTAL_INCREMENT = 2,     // 累计增量
        TOTAL_DECREMENT = 3,     // 累计减量
        HISTORICAL_PRESENCE = 4, // 历史存在
    };

    enum temporalScope{     // 给外部引用
        NULL_TEMPORAL = 0,     // 空
        FULL_CYCLE = 1,     // 全周期
        TASK_CYCLE = 2,     // 任务周期
        ROLLING_WINDOW = 3, // 滚动周期
    };

    // 规则条件结构体
    struct RuleCondition {
        int min_value;          // 最小值（对于范围判断）或比较值
        int max_value;          // 最大值（对于范围判断）
        MatchType match_type;     // 匹配类型
        metricType metric_type;   // 指标类型
        temporalScope temporal_scope; // 时间范围
        int scope_value;          // 时间范围值
        std::string datatype;         // 条件名称
        database::data_type data_type;
        RuleCondition(std::string datatype) : min_value(0), max_value(0), match_type(IGNORE), metric_type(CURRENT_VALUE), temporal_scope(FULL_CYCLE), scope_value(0), datatype(datatype) {
            if(datatype == "game_progress") data_type = database::GAME_PROGRESS;
            else if(datatype == "stage_remain_time") data_type = database::STAGE_REMAIN_TIME;
            else if(datatype == "own_robot_HP") data_type = database::OWN_ROBOT_HP;
            else if(datatype == "own_base_HP") data_type = database::OWN_BASE_HP;
            else if(datatype == "enemy_base_HP") data_type = database::ENEMY_BASE_HP;
            else if(datatype == "event_data") data_type = database::EVENT_DATA;
            else if(datatype == "current_HP") data_type = database::CURRENT_HP;
            else if(datatype == "shoot_num") data_type = database::SHOOT_NUM;
            else if(datatype == "vision_status") data_type = database::VISION_STATUS;
            else if(datatype == "HP_deduction_reason") data_type = database::HP_DEDUCTION_REASON;
            else if(datatype == "armor_id") data_type = database::ARMOR_ID;
            else if(datatype == "rfid_status") data_type = database::RFID_STATUS;
            else if(datatype == "last_task_id") data_type = database::LAST_TASK_ID;
        }
        void change_value(int min_value, int max_value, int match_type, int metric_type, int temporal_scope, int scope_value){
            this->min_value = min_value;
            this->max_value = max_value;
            this->match_type = (MatchType)match_type;
            this->metric_type = (metricType)metric_type;
            this->temporal_scope = (temporalScope)temporal_scope;
            this->scope_value = scope_value;
        }
    };

    // 决策规则结构体
    class DecisionRules {
    public:
        std::vector<RuleCondition> conditions;
        int condition_count = 0;
        void add_condition(RuleCondition condition){
            conditions.push_back(condition);
        }
    };

}; // namespace rules

#endif // RULES_H