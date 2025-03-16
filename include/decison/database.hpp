#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <unordered_map>
#include <atomic>
#include <shared_mutex>

#include <ros/ros.h>
#include <roborts_msgs/driver.h>

// 容器预分配大小
#define container_size 300 * 30

namespace database
{
    struct data
    {
        std::vector<int> raw_data;        // 原始数据
        std::vector<int> prefix_inc;      // 预处理累计增加
        std::vector<int> prefix_dec;      // 预处理累计减少
        std::vector<ros::Time> timestamp; // 时间戳索引
        std::unordered_map<int, std::vector<ros::Time>> timestamp_map; // 记录每个数据对应的所有时间戳
        data(){
            raw_data.reserve(container_size);
            prefix_inc.reserve(container_size);
            prefix_dec.reserve(container_size);
            timestamp.reserve(container_size);
            timestamp_map.reserve(container_size);
        }
    };

    enum metricType{
        CURRENT_VALUE = 1,       // 当前数值
        TOTAL_INCREMENT = 2,     // 累计增量
        TOTAL_DECREMENT = 3,     // 累计减量
        HISTORICAL_PRESENCE = 4, // 历史存在
    };

    enum temporalScope{     // 给外部引用
        FULL_CYCLE = 1,     // 全周期
        TASK_CYCLE = 2,     // 任务周期
        ROLLING_WINDOW = 3, // 滚动周期
    };

    enum prefixType{
        INCREASE = 1,
        DECREASE = 2,
        SUM = 3
    };
    enum data_type{
        GAME_PROGRESS,
        STAGE_REMAIN_TIME,
        OWN_ROBOT_HP,
        OWN_BASE_HP,
        ENEMY_BASE_HP,
        EVENT_DATA,
        CURRENT_HP,
        SHOOT_NUM,
        VISION_STATUS,
        HP_DEDUCTION_REASON,
        ARMOR_ID,
        RFID_STATUS
    };

    struct DataContainer
    {
        data game_progress;
        data stage_remain_time;
        data own_robot_HP;
        data own_base_HP;
        data enemy_base_HP;
        data event_data;
        data current_HP;
        data shoot_num;
        data vision_status;
        data HP_deduction_reason;
        data armor_id;
        data rfid_status;
    };
    class Database{
        private:
            DataContainer dataContainer_;
            
            // 自旋锁 (使用std::atomic_flag实现)
            std::atomic_flag spinlock = ATOMIC_FLAG_INIT;
            
        public:
            void update_data(roborts_msgs::driver msg);
            bool check_data(data_type datatype, ros::Time end, ros::Duration duration, metricType type, int min_value = 0, int max_value = 0);
        private:
            bool confirm_metric_compliance(data_type datatype, ros::Time end, ros::Duration duration, metricType type, int min_value = 0, int max_value = 0);
            void preprocess_data(data &data, ros::Time stamp, int value);
            int get_duration_data(data &data, ros::Time end, ros::Duration duration, prefixType type);
            int get_current_data(data &data);
            bool hasValueDuring(data &data, ros::Time end, ros::Duration duration, int min_value = 0, int max_value = 0);
            std::pair<int, int> Time_get_index(data &data, ros::Time end, ros::Duration duration);
            data &get_data_base(data_type datatype);
    };

    data &Database::get_data_base(data_type datatype){
        // 获取数据
        switch (datatype){
            case GAME_PROGRESS:return dataContainer_.game_progress;
            case STAGE_REMAIN_TIME:return dataContainer_.stage_remain_time;
            case OWN_ROBOT_HP:return dataContainer_.own_robot_HP;
            case OWN_BASE_HP:return dataContainer_.own_base_HP;
            case ENEMY_BASE_HP:return dataContainer_.enemy_base_HP;
            case EVENT_DATA:return dataContainer_.event_data;
            case CURRENT_HP:return dataContainer_.current_HP;
            case SHOOT_NUM:return dataContainer_.shoot_num;
            case VISION_STATUS:return dataContainer_.vision_status;
            case HP_DEDUCTION_REASON:return dataContainer_.HP_deduction_reason;
            case ARMOR_ID:return dataContainer_.armor_id;
            case RFID_STATUS:return dataContainer_.rfid_status;
            default: throw std::runtime_error("Invalid data type");
        }
    }
    void Database::update_data(roborts_msgs::driver msg){
        // 使用自旋锁
        while (spinlock.test_and_set(std::memory_order_acquire));
        
        // 更新数据
        preprocess_data(dataContainer_.game_progress, msg.header.stamp, msg.game_progress);
        preprocess_data(dataContainer_.stage_remain_time, msg.header.stamp, msg.stage_remain_time);
        preprocess_data(dataContainer_.own_robot_HP, msg.header.stamp, msg.own_robot_HP);
        preprocess_data(dataContainer_.own_base_HP, msg.header.stamp, msg.own_base_HP);
        preprocess_data(dataContainer_.enemy_base_HP, msg.header.stamp, msg.enemy_base_HP);
        preprocess_data(dataContainer_.event_data, msg.header.stamp, msg.event_data);
        preprocess_data(dataContainer_.current_HP, msg.header.stamp, msg.current_HP);
        preprocess_data(dataContainer_.shoot_num, msg.header.stamp, msg.shoot_num);
        preprocess_data(dataContainer_.vision_status, msg.header.stamp, msg.vision_status);
        preprocess_data(dataContainer_.HP_deduction_reason, msg.header.stamp, msg.HP_deduction_reason);
        preprocess_data(dataContainer_.armor_id, msg.header.stamp, msg.armor_id);
        preprocess_data(dataContainer_.rfid_status, msg.header.stamp, msg.rfid_status);
        
        // 释放自旋锁
        spinlock.clear(std::memory_order_release);
    }
    
    void Database::preprocess_data(data &data, ros::Time stamp, int value){
        // 预处理数据
        if (!data.raw_data.empty()){
            if (stamp <= data.timestamp.back()){
                std::cout << "stamp <= data.timestamp.back()" << std::endl;
                return;
            }
        }// 避免时间戳回退

        if (data.raw_data.empty()){
            data.prefix_dec.push_back(0);
            data.prefix_inc.push_back(0);
        }else if (value > data.raw_data.back()){
            data.prefix_inc.push_back(value - data.raw_data.back() + data.prefix_inc.back());
            data.prefix_dec.push_back(data.prefix_dec.back());
        }else if (value < data.raw_data.back()){
            data.prefix_dec.push_back(data.raw_data.back() - value + data.prefix_dec.back());
            data.prefix_inc.push_back(data.prefix_inc.back());
        }else{
            data.prefix_inc.push_back(data.prefix_inc.back());
            data.prefix_dec.push_back(data.prefix_dec.back());
        }
        
        data.raw_data.push_back(value);
        data.timestamp.push_back(stamp);
        data.timestamp_map[value].push_back(stamp);
    }

    int Database::get_duration_data(data &data, ros::Time end, ros::Duration duration, prefixType type){
        // 获取一段时间内数据的变化量
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        // if (end > data.timestamp.back())throw std::runtime_error("error: end is out of range");
        if (end - duration < data.timestamp.front())throw std::runtime_error("error: duration is out of range");

        std::pair<int, int> index = Time_get_index(data, end, duration);
        int start_index = index.first;
        int end_index = index.second;

        if (type == INCREASE)return data.prefix_inc[end_index] - data.prefix_inc[start_index];
        else if (type == DECREASE)return data.prefix_dec[end_index] - data.prefix_dec[start_index];
        else if (type == SUM)return data.prefix_inc[end_index] - data.prefix_inc[start_index] + data.prefix_dec[end_index] - data.prefix_dec[start_index];
        else throw std::runtime_error("error: invalid type");
    }

    int Database::get_current_data(data &data){
        // 获取当前数据
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        return data.raw_data.back();
    }

    bool Database::hasValueDuring(data &data, ros::Time end, ros::Duration duration, int min_value, int max_value){
        // 判断在一段时间内是否有数据
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        // if (end > data.timestamp.back())throw std::runtime_error("error: end is out of range");
        if (end - duration < data.timestamp.front())throw std::runtime_error("error: duration is out of range");
        if (min_value == max_value)
        {// 利用哈希表快速查找
            std::vector<ros::Time> timestamps = data.timestamp_map[min_value];
            if (timestamps.empty())return false;
            for (auto &timestamp : timestamps){
                if (timestamp <= end && timestamp >= end - duration)return true;
            }
            return false;
        }else{
            std::pair<int, int> index = Time_get_index(data, end, duration);
            int start_index = index.first;
            int end_index = index.second;
            for (int i = start_index; i < end_index; i++){
                if (data.raw_data[i] >= min_value && data.raw_data[i] <= max_value)return true;
            }
            return false;
        }
    }

    std::pair<int, int> Database::Time_get_index(data &data, ros::Time end, ros::Duration duration){
        if (data.timestamp.empty()) throw std::runtime_error("error: timestamp is empty");
        
        // 默认使用最后一个索引
        int end_index = data.timestamp.size() - 1;
        
        // 如果end比最后时间戳大，但在可接受范围内，仍使用最后索引
        // 否则使用二分查找
        if (end - data.timestamp.back() > ros::Duration(0.1)){
            auto it = std::upper_bound(data.timestamp.begin(), data.timestamp.end(), end);
            if (it == data.timestamp.end()) {
                end_index = data.timestamp.size() - 1;  // 使用最后一个索引
            } else {
                end_index = it - data.timestamp.begin() - 1;  // 减1获取小于等于end的最大索引
            }
        }
        
        // 查找起始索引
        int start_index = std::lower_bound(data.timestamp.begin(), data.timestamp.end(), end - duration) - data.timestamp.begin();
        
        // 边界检查
        if (start_index > end_index) throw std::runtime_error("error: start_index is out of range");
        if (end_index >= data.timestamp.size()) throw std::runtime_error("error: end_index is out of range");
        
        return std::make_pair(start_index, end_index);
    }


    bool Database::confirm_metric_compliance(data_type datatype, ros::Time end, ros::Duration duration, metricType type, int min_value, int max_value){
        try {
            data &data = get_data_base(datatype);
            if (data.raw_data.empty()) return false;
            if (type == CURRENT_VALUE)
            {
                int result = get_current_data(data);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == TOTAL_INCREMENT)
            {
                int result = get_duration_data(data, end, duration, prefixType::INCREASE);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == TOTAL_DECREMENT)
            {
                int result = get_duration_data(data, end, duration, prefixType::DECREASE);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == HISTORICAL_PRESENCE)
            {
                return hasValueDuring(data, end, duration, min_value, max_value);
            }
        } catch (const std::runtime_error& e) {
            std::cout << e.what() << std::endl;
            std::cout << "error: not found data" << std::endl;
        }
        return false;
    }

    bool Database::check_data(data_type datatype, ros::Time end, ros::Duration duration, metricType type, int min_value, int max_value){
        // 使用自旋锁
        while (spinlock.test_and_set(std::memory_order_acquire));
        bool result = confirm_metric_compliance(datatype, end, duration, type, min_value, max_value);
        // 释放自旋锁
        spinlock.clear(std::memory_order_release);
        return result;
    }

} // namespace database

#endif // DATABASE_H