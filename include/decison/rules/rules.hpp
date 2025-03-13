#ifndef RULES_H
#define RULES_H

#include <iostream>
#include <vector>
#include <functional>
#include "roborts_msgs/driver.h"

namespace rules {
    // 匹配类型枚举
    enum MatchType {
        GREATER_THAN,     // 大于
        LESS_THAN,        // 小于
        EQUAL_TO,         // 等于
        IN_RANGE,         // 在范围内
        IGNORE            // 忽略此条件
    };

    // 规则条件结构体
    struct RuleCondition {
        float min_value;          // 最小值（对于范围判断）或比较值
        float max_value;          // 最大值（对于范围判断）
        MatchType match_type;     // 匹配类型
        std::string type;         // 条件名称
        // 构造函数，简化创建条件
        RuleCondition(float value, MatchType type) 
            : min_value(value), max_value(0), match_type(type) {}
        
        RuleCondition(float min, float max, MatchType type = IN_RANGE) 
            : min_value(min), max_value(max), match_type(type) {}
        
        // 默认构造函数，创建IGNORE类型条件
        RuleCondition() : min_value(0), max_value(0), match_type(IGNORE) {}
        RuleCondition(std::string type) : min_value(0), max_value(0), match_type(IGNORE), type(type) {}
        // 判断条件是否匹配
        bool matches(float value) const {
            switch (match_type) {
                case GREATER_THAN:
                    return value > min_value;
                case LESS_THAN:
                    return value < min_value;
                case EQUAL_TO:
                    return value == min_value;
                case IN_RANGE:
                    return value >= min_value && value <= max_value;
                case IGNORE:
                    return true;
                default:
                    std::cerr << "未知的匹配类型" << std::endl;
                    return false;
            }
        }
    };

    // 决策规则结构体
    class DecisionRules {
    public:
        std::vector<RuleCondition> conditions;
        int condition_count = 0;

    public:
        DecisionRules(){
            conditions.push_back(RuleCondition("game_progress"));
            conditions.push_back(RuleCondition("stage_remain_time"));
            conditions.push_back(RuleCondition("own_robot_HP"));
            conditions.push_back(RuleCondition("own_base_HP"));
            conditions.push_back(RuleCondition("enemy_base_HP"));
            conditions.push_back(RuleCondition("event_data"));
            conditions.push_back(RuleCondition("current_HP"));
            conditions.push_back(RuleCondition("shoot_num"));
            conditions.push_back(RuleCondition("vision_status"));
            conditions.push_back(RuleCondition("HP_deduction_reason"));
            conditions.push_back(RuleCondition("armor_id"));
            conditions.push_back(RuleCondition("rfid_status"));
        }
    };

    // 匹配规则的函数
    bool match_rules(const DecisionRules& rule, roborts_msgs::driver *data) {
        // 检查所有条件是否匹配
        return rule.conditions[0].matches(data->game_progress) &&
               rule.conditions[1].matches(data->stage_remain_time) &&
               rule.conditions[2].matches(data->own_robot_HP) &&
               rule.conditions[3].matches(data->own_base_HP) &&
               rule.conditions[4].matches(data->enemy_base_HP) &&
               rule.conditions[5].matches(data->event_data) &&
               rule.conditions[6].matches(data->current_HP) &&
               rule.conditions[7].matches(data->vision_status) &&
               rule.conditions[8].matches(data->armor_id) &&
               rule.conditions[9].matches(data->HP_deduction_reason) &&
               rule.conditions[10].matches(data->shoot_num) &&
               rule.conditions[11].matches(data->rfid_status);
    }
    
}; // namespace rules

#endif // RULES_H