#ifndef RULES_H
#define RULES_H

#include <iostream>
#include "roborts_msgs/driver.h"

namespace rules{
    enum Match{
        outweigh,
        underweight,
        equal_weight,
        interval,
        ignore
    };


    struct decison_rules
    {
        //比赛状态
        std::pair<std::pair<float,float>,Match> game_progress;

        std::pair<std::pair<float,float>,Match> stage_remain_time;
        std::pair<std::pair<float,float>,Match> own_robot_HP;
        std::pair<std::pair<float,float>,Match> own_base_HP;
        std::pair<std::pair<float,float>,Match> enemy_base_HP;
        std::pair<std::pair<float,float>,Match> event_data;
        std::pair<std::pair<float,float>,Match> current_HP;
        std::pair<std::pair<float,float>,Match> shoot_num;

        std::pair<std::pair<float,float>,Match> vision_status;
        std::pair<std::pair<float,float>,Match> HP_deduction_reason;
        std::pair<std::pair<float,float>,Match> armor_id;
        std::pair<std::pair<float,float>,Match> rfid_status;
    };


    // 匹配规则的函数
    bool match_rules(decison_rules rule, roborts_msgs::driver::ConstPtr data){
        // 计算匹配结果
        bool result = 
        math(rule.game_progress,data->game_progress)*
        math(rule.stage_remain_time,data->stage_remain_time)*
        math(rule.own_robot_HP,data->own_robot_HP)*
        math(rule.own_base_HP,data->own_base_HP)*
        math(rule.enemy_base_HP,data->enemy_base_HP)*
        math(rule.event_data,data->event_data)*
        math(rule.current_HP,data->current_HP)*
        math(rule.vision_status,data->vision_status)*
        math(rule.armor_id,data->armor_id)*
        math(rule.HP_deduction_reason,data->HP_deduction_reason)*
        math(rule.shoot_num,data->shoot_num)*
        math(rule.rfid_status,data->rfid_status);
        
        return result;
    }

    bool math(std::pair<std::pair<float,float>,Match> rule, float data){
        switch (rule.second)
        {
        case outweigh:
            return rule.first.first > data;
        case underweight:
            return rule.first.first < data;
        case equal_weight:
            return rule.first.first == data;
        case ignore:
            return true;
        case interval:
            return data > rule.first.first && data < rule.first.second;
        default:
            std::cerr << "异常的匹配规则" << std::endl;
            return false;
        }
    }
}; // namespace rules

#endif // RULES_H