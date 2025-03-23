#ifndef DECISION_DEBUGPAGE_H
#define DECISION_DEBUGPAGE_H

#include "gui/uitool.h"
#include "gui/ui/debugpage_ui.h"
#include "roborts_msgs/driver.h"
#include "ros/ros.h"
#include <thread>

class DebugPage : public Ui_DEBUG_PAGE, public QObject{
public:
    void init();
    void loop();
    
    void pub_manage(int value);
    void sub_manage(int value);
    void updateInfo(const roborts_msgs::driver::ConstPtr &msg);
    void updateSimdata();
    
private:
    std::unique_ptr<std::thread> loop_thread; //循环线程
    colorStyleSheet color; //颜色样式
    std::mutex mutex_;
    roborts_msgs::driver simdata;
    std::unique_ptr<ros::Subscriber> sub_driver;
    std::unique_ptr<ros::Publisher> pub_simdata;
    ros::NodeHandle nh;
};

void DebugPage::init(){
    //monitor栏初始化
    changeLabelText(this->monitor_game_progress_state, "无", color.grey);
    changeLabelText(this->monitor_HP_deduction_reason_state, "无", color.grey);
    changeLabelText(this->monitor_armor_id_state, "无", color.grey);
    changeLabelText(this->monitor_event_data_state, "无", color.grey);

    changeLabelText(this->monitor_task_id_state, "无", color.grey);
    changeLabelText(this->monitor_goal_pose_state, "无", color.grey);
    changeLabelText(this->monitor_vision_status_sate, "无", color.grey);
    changeLabelText(this->monitor_enemy_pose_state, "无", color.grey);

    //progress栏初始化
    this->progress_stage_remain_time_bar_value->setValue(0);
    this->progress_own_robot_HP_bar_value->setValue(0);
    this->progress_own_base_HP_bar_value->setValue(0);
    this->progress_enemy_base_HP_bar_value->setValue(0);
    this->progress_shoot_num_bar_value->setValue(0);
    this->progress_nav_state_value->setValue(0);

    //rules栏初始化
    changeLabelText(this->rules_date_state, "无", color.grey);
    changeLabelText(this->rules_nums_value, "无", color.grey);
    changeLabelText(this->rules_version_state, "无", color.grey);

    //log栏初始化
    this->logQS_LOG_TEXT->clear();
    this->logQS_LOG_TEXT->append("日志初始化完成");

    //按钮初始化
    QObject::connect(this->Sim_Data_change_QP, &QPushButton::clicked, this, &DebugPage::updateSimdata);

    //滑动开关初始化
    QObject::connect(this->debug_send_QSL, &QSlider::valueChanged, this, &DebugPage::pub_manage);
    QObject::connect(this->debug_listen_QSL, &QSlider::valueChanged, this, &DebugPage::sub_manage);

    loop_thread = std::make_unique<std::thread>(&DebugPage::loop, this); // 启动线程
}


void DebugPage::loop(){
    while(ros::ok()){
        {
            std::lock_guard<std::mutex> lock(mutex_); //加锁
            simdata.header.stamp = ros::Time::now();
            if (this->pub_simdata != nullptr) this->pub_simdata->publish(simdata);  //发布
        }
        ros::Duration(0.1).sleep();
    }
}

void DebugPage::updateInfo(const roborts_msgs::driver::ConstPtr &msg){
    //monitor栏初始化
    if(msg->game_progress == 4){
        changeLabelText(this->monitor_game_progress_state, "进行中", color.green);
    }else{
        changeLabelText(this->monitor_game_progress_state, "未开始", color.red);
    }

    if(msg->HP_deduction_reason == 255){
        changeLabelText(this->monitor_HP_deduction_reason_state, "无", color.grey);
    }else{
        changeLabelText(this->monitor_HP_deduction_reason_state, 
        std::to_string(msg->HP_deduction_reason).c_str(), color.red);
    }

    if(msg->armor_id == 255){
        changeLabelText(this->monitor_armor_id_state, "无", color.grey);
    }else{
        changeLabelText(this->monitor_armor_id_state, 
        std::to_string(msg->armor_id).c_str(), color.red);
    }

    if (msg->event_data == -1){
        changeLabelText(this->monitor_event_data_state, "无", color.grey);
    }else{
        changeLabelText(this->monitor_event_data_state, 
        std::to_string(msg->event_data).c_str(), color.red);
    }
    
    // changeLabelText(this->monitor_task_id_state, "无", color.grey);

    // changeLabelText(this->monitor_goal_pose_state, "无", color.grey);
    if(msg->vision_status == 0){
        changeLabelText(this->monitor_vision_status_sate, "无", color.grey);
    }else{
        changeLabelText(this->monitor_vision_status_sate, "检测到敌方", color.red);
    }
    // changeLabelText(this->monitor_enemy_pose_state, "无", color.grey);

    //progress栏初始化
    this->progress_stage_remain_time_bar_value->setValue(msg->stage_remain_time);
    this->progress_own_robot_HP_bar_value->setValue(msg->current_HP);
    this->progress_own_base_HP_bar_value->setValue(msg->own_base_HP);
    this->progress_enemy_base_HP_bar_value->setValue(msg->enemy_base_HP);
    this->progress_shoot_num_bar_value->setValue(msg->shoot_num);
    
    // this->progress_nav_state_value->setValue(0);
}

void DebugPage::updateSimdata(){
    std::lock_guard<std::mutex> lock(mutex_);
    simdata.header.stamp = ros::Time::now();
    simdata.armor_id = this->Sim_Data_armor_id_QL->text().toInt();
    simdata.enemy_base_HP = this->Sim_Data_enemy_base_HP_QL->text().toInt();
    simdata.event_data = this->Sim_Data_event_data_QL->text().toInt();
    simdata.game_progress = this->Sim_Data_game_progress_QL->text().toInt();
    simdata.HP_deduction_reason = this->Sim_Data_HP_deduction_reason_QL->text().toInt();
    simdata.own_base_HP = this->Sim_Data_own_base_HP_QL->text().toInt();
    simdata.own_robot_HP = this->Sim_Data_own_robot_HP_QL->text().toInt();
    simdata.shoot_num = this->Sim_Data_shoot_num_QL->text().toInt();
    simdata.stage_remain_time = this->Sim_Data_stage_remain_time_QL->text().toInt();
    simdata.vision_status = this->Sim_Data_vision_status_QL->text().toInt();

    simdata.current_HP = simdata.own_robot_HP;
}

void DebugPage::pub_manage(int value){
    try
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (value == 1){
            this->pub_simdata = std::make_unique<ros::Publisher>
            (nh.advertise<roborts_msgs::driver>
            (this->pub_topic_QL->text().toStdString().c_str(), 1));
            this->logQS_LOG_TEXT->append(QString::fromStdString(std::string("开始发布话题")));
        }else{
            this->pub_simdata = nullptr;
        }
    }
    catch(const std::exception& e)
    {
        this->logQS_LOG_TEXT->append(QString::fromStdString
        (std::string("无法发布到话题 :") + e.what()));
        this->debug_send_QSL->setValue(0);
    }   
}

void DebugPage::sub_manage(int value){
    try
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (value == 1){
            this->sub_driver = std::make_unique<ros::Subscriber>
            (nh.subscribe(this->pub_topic_QL->text().toStdString().c_str(), 1, &DebugPage::updateInfo, this));
            this->logQS_LOG_TEXT->append(QString::fromStdString(std::string("开始订阅话题")));
        }else{
            this->sub_driver = nullptr;
        }
    }
    catch(const std::exception& e)
    {
        this->logQS_LOG_TEXT->append(QString::fromStdString(
        std::string("无法订阅到话题 :") + e.what()));
        this->debug_listen_QSL->setValue(0);
    }   
}

#endif // DECISION_DEBUGPAGE_H
