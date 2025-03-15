#ifndef RULES_H
#define RULES_H

#include <iostream>
#include <vector>
#include <functional>
#include "roborts_msgs/driver.h"

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
        RuleCondition(std::string datatype) : min_value(0), max_value(0), match_type(IGNORE), metric_type(CURRENT_VALUE), temporal_scope(FULL_CYCLE), scope_value(0), datatype(datatype) {}
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