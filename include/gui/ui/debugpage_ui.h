/********************************************************************************
** Form generated from reading UI file 'debugPSxLZG.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef DEBUGPSXLZG_H
#define DEBUGPSXLZG_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_DEBUG_PAGE
{
public:
    QWidget *centralwidget;
    QWidget *progress_bar;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *progress_bar_QV;
    QHBoxLayout *progress_stage_remain_time_bar;
    QLabel *progress_stage_remain_time_bar_title;
    QProgressBar *progress_stage_remain_time_bar_value;
    QHBoxLayout *progress_own_robot_HP_bar;
    QLabel *progress_own_robot_HP_bar_title;
    QProgressBar *progress_own_robot_HP_bar_value;
    QHBoxLayout *progress_own_base_HP_bar;
    QLabel *progress_own_base_HP_bar_title;
    QProgressBar *progress_own_base_HP_bar_value;
    QHBoxLayout *progress_enemy_base_HP_bar;
    QLabel *progress_enemy_base_HP_bar_title;
    QProgressBar *progress_enemy_base_HP_bar_value;
    QHBoxLayout *progress_shoot_num_bar;
    QLabel *progress_shoot_num_bar_title;
    QProgressBar *progress_shoot_num_bar_value;
    QHBoxLayout *progress_nav_state;
    QLabel *progress_nav_state_title;
    QProgressBar *progress_nav_state_value;
    QWidget *home;
    QWidget *horizontalLayoutWidget_12;
    QHBoxLayout *home_QH;
    QPushButton *home_page;
    QPushButton *rules_page;
    QPushButton *debug_page;
    QPushButton *setting_page;
    QWidget *monitor_board;
    QWidget *horizontalLayoutWidget_7;
    QHBoxLayout *monitor_board_2;
    QWidget *monitor_task_id_QW;
    QWidget *verticalLayoutWidget_9;
    QVBoxLayout *monitor_task_id_QV;
    QLabel *monitor_task_id_title;
    QLabel *monitor_task_id_state;
    QWidget *monitor_goal_pose_QW;
    QWidget *verticalLayoutWidget_8;
    QVBoxLayout *monitor_goal_pose_QV;
    QLabel *monitor_goal_pose_title;
    QLabel *monitor_goal_pose_state;
    QWidget *monitor_vision_status_QW;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *monitor_vision_status_QV;
    QLabel *monitor_vision_status_title;
    QLabel *monitor_vision_status_sate;
    QWidget *monitor_enemy_pose_QW;
    QWidget *verticalLayoutWidget_10;
    QVBoxLayout *monitor_enemy_pose_QV;
    QLabel *monitor_enemy_pose_title;
    QLabel *monitor_enemy_pose_state;
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *monitor_board_1;
    QWidget *monitor_game_progress_QW;
    QWidget *verticalLayoutWidget_5;
    QVBoxLayout *monitor_game_progress_QV;
    QLabel *monitor_game_progress_title;
    QLabel *monitor_game_progress_state;
    QWidget *monitor_HP_deduction_reason_QW;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *monitor_HP_deduction_reason_QV;
    QLabel *monitor_HP_deduction_reason_title;
    QLabel *monitor_HP_deduction_reason_state;
    QWidget *monitor_armor_id_QW;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *monitor_armor_id_QV;
    QLabel *monitor_armor_id_title;
    QLabel *monitor_armor_id_state;
    QWidget *monitor_event_data_QW;
    QWidget *verticalLayoutWidget_7;
    QVBoxLayout *monitor_event_data_QV;
    QLabel *monitor_event_data_title;
    QLabel *monitor_event_data_state;
    QScrollArea *logQS;
    QWidget *logQS_QW;
    QTextBrowser *logQS_LOG_TEXT;
    QWidget *Sim_Data_QW;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *Sim_Data_QV_1;
    QWidget *Sim_Data_game_progress_QW;
    QWidget *verticalLayoutWidget_16;
    QVBoxLayout *Sim_Data_game_progress_QV;
    QLabel *Sim_Data_game_progress_title;
    QLineEdit *Sim_Data_game_progress_QL;
    QWidget *Sim_Data_own_robot_HP_QW;
    QWidget *verticalLayoutWidget_18;
    QVBoxLayout *Sim_Data_own_robot_HP_QV;
    QLabel *Sim_Data_own_robot_HP_title;
    QLineEdit *Sim_Data_own_robot_HP_QL;
    QWidget *Sim_Data_enemy_base_HP_QW;
    QWidget *verticalLayoutWidget_15;
    QVBoxLayout *Sim_Data_enemy_base_HP_QV;
    QLabel *Sim_Data_enemy_base_HP_title;
    QLineEdit *Sim_Data_enemy_base_HP_QL;
    QWidget *Sim_Data_HP_deduction_reason_QW;
    QWidget *verticalLayoutWidget_23;
    QVBoxLayout *Sim_Data_HP_deduction_reason_QV;
    QLabel *Sim_Data_HP_deduction_reason_title;
    QLineEdit *Sim_Data_HP_deduction_reason_QL;
    QWidget *Sim_Data_shoot_num_QW;
    QWidget *verticalLayoutWidget_21;
    QVBoxLayout *Sim_Data_shoot_num_QV;
    QLabel *Sim_Data_shoot_num_title;
    QLineEdit *Sim_Data_shoot_num_QL;
    QWidget *verticalLayoutWidget_11;
    QVBoxLayout *Sim_Data_QV_2;
    QWidget *Sim_Data_stage_remain_time_QW;
    QWidget *verticalLayoutWidget_17;
    QVBoxLayout *Sim_Data_stage_remain_time_QV;
    QLabel *Sim_Data_stage_remain_time_title;
    QLineEdit *Sim_Data_stage_remain_time_QL;
    QWidget *Sim_Data_vision_status_QW;
    QWidget *verticalLayoutWidget_22;
    QVBoxLayout *Sim_Data_vision_status_QV;
    QLabel *Sim_Data_vision_status_title;
    QLineEdit *Sim_Data_vision_status_QL;
    QWidget *Sim_Data_own_base_HP_QW;
    QWidget *verticalLayoutWidget_19;
    QVBoxLayout *Sim_Data_own_base_HP_QV;
    QLabel *Sim_Data_own_base_HP_title;
    QLineEdit *Sim_Data_own_base_HP_QL;
    QWidget *Sim_Data_event_data_QW;
    QWidget *verticalLayoutWidget_20;
    QVBoxLayout *Sim_Data_event_data_QV;
    QLabel *Sim_Data_event_data_title;
    QLineEdit *Sim_Data_event_data_QL;
    QWidget *Sim_Data_armor_id_QW;
    QWidget *verticalLayoutWidget_24;
    QVBoxLayout *Sim_Data_armor_id_QV;
    QLabel *Sim_Data_armor_id_title;
    QLineEdit *Sim_Data_armor_id_QL;
    QLabel *Sim_Data_TITLE;
    QLabel *Sim_Data_tips;
    QPushButton *Sim_Data_change_QP;
    QWidget *DEBUG;
    QLabel *DEBUG_TITLE;
    QWidget *horizontalLayoutWidget_8;
    QHBoxLayout *rules_QH;
    QWidget *rules_date_QW;
    QWidget *verticalLayoutWidget_12;
    QVBoxLayout *rules_date_QV;
    QLabel *rules_date_title;
    QLabel *rules_date_state;
    QWidget *rules_version_QW;
    QWidget *verticalLayoutWidget_13;
    QVBoxLayout *rules_version_QV;
    QLabel *rules_version_title;
    QLabel *rules_version_state;
    QWidget *rules_nums_QW;
    QWidget *verticalLayoutWidget_14;
    QVBoxLayout *rules_nums_QV;
    QLabel *rules_nums_title;
    QLabel *rules_nums_value;
    QLineEdit *rules_path_QL;
    QPushButton *refresh_rules_QP;
    QPushButton *reload_rules_QP;
    QLabel *rules_path_tips;
    QWidget *goal_pose_debug_QW;
    QLabel *goal_pose_title;
    QWidget *horizontalLayoutWidget_5;
    QHBoxLayout *goal_pose_QH;
    QLineEdit *goal_pose_X;
    QLineEdit *goal_pose_Y;
    QLineEdit *goal_pose_YAW;
    QLabel *goal_pose_tips;
    QWidget *horizontalLayoutWidget_6;
    QHBoxLayout *goal_pose_send_way_QH;
    QLabel *goal_pose_send_way_title;
    QSlider *goal_pose_send_way_QSL;
    QPushButton *goal_pose_send_QP;
    QLabel *goal_pose_send_way_tips;
    QWidget *datadebug_QW;
    QWidget *horizontalLayoutWidget_4;
    QHBoxLayout *debug_swtich_QH;
    QHBoxLayout *debug_send_QH;
    QLabel *debug_send_title;
    QSlider *debug_send_QSL;
    QHBoxLayout *debug_listen_QH;
    QLabel *debug_listen_title;
    QSlider *debug_listen_QSL;
    QLineEdit *pub_topic_QL;
    QLabel *datadebug_title;
    QLabel *debug_tips;

    void setupUi(QMainWindow *DEBUG_PAGE)
    {
        if (DEBUG_PAGE->objectName().isEmpty())
            DEBUG_PAGE->setObjectName(QStringLiteral("DEBUG_PAGE"));
        DEBUG_PAGE->resize(1920, 1080);
        DEBUG_PAGE->setMinimumSize(QSize(1920, 1080));
        DEBUG_PAGE->setMaximumSize(QSize(1920, 1080));
        QFont font;
        font.setFamily(QString::fromUtf8("\345\276\256\350\275\257\351\233\205\351\273\221"));
        font.setBold(true);
        font.setWeight(75);
        DEBUG_PAGE->setFont(font);
        DEBUG_PAGE->setLayoutDirection(Qt::LeftToRight);
        DEBUG_PAGE->setIconSize(QSize(40, 40));
        DEBUG_PAGE->setToolButtonStyle(Qt::ToolButtonFollowStyle);
        DEBUG_PAGE->setAnimated(true);
        centralwidget = new QWidget(DEBUG_PAGE);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        centralwidget->setMinimumSize(QSize(1920, 1080));
        centralwidget->setMaximumSize(QSize(1920, 1080));
        centralwidget->setStyleSheet(QStringLiteral("background-color: rgb(240, 240, 240);"));
        progress_bar = new QWidget(centralwidget);
        progress_bar->setObjectName(QStringLiteral("progress_bar"));
        progress_bar->setGeometry(QRect(20, 350, 800, 320));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(progress_bar->sizePolicy().hasHeightForWidth());
        progress_bar->setSizePolicy(sizePolicy);
        progress_bar->setLayoutDirection(Qt::LeftToRight);
        progress_bar->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        verticalLayoutWidget = new QWidget(progress_bar);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(0, 0, 801, 321));
        progress_bar_QV = new QVBoxLayout(verticalLayoutWidget);
        progress_bar_QV->setSpacing(5);
        progress_bar_QV->setObjectName(QStringLiteral("progress_bar_QV"));
        progress_bar_QV->setSizeConstraint(QLayout::SetDefaultConstraint);
        progress_bar_QV->setContentsMargins(20, 20, 20, 20);
        progress_stage_remain_time_bar = new QHBoxLayout();
        progress_stage_remain_time_bar->setSpacing(25);
        progress_stage_remain_time_bar->setObjectName(QStringLiteral("progress_stage_remain_time_bar"));
        progress_stage_remain_time_bar_title = new QLabel(verticalLayoutWidget);
        progress_stage_remain_time_bar_title->setObjectName(QStringLiteral("progress_stage_remain_time_bar_title"));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font1.setPointSize(11);
        progress_stage_remain_time_bar_title->setFont(font1);
        progress_stage_remain_time_bar_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_stage_remain_time_bar->addWidget(progress_stage_remain_time_bar_title);

        progress_stage_remain_time_bar_value = new QProgressBar(verticalLayoutWidget);
        progress_stage_remain_time_bar_value->setObjectName(QStringLiteral("progress_stage_remain_time_bar_value"));
        QFont font2;
        font2.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        progress_stage_remain_time_bar_value->setFont(font2);
        progress_stage_remain_time_bar_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #7FB0FF,\n"
"        stop: 0.5 #7FB0FF,\n"
"        stop: 1 #7FB0FF\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_stage_remain_time_bar_value->setMaximum(500);
        progress_stage_remain_time_bar_value->setValue(345);
        progress_stage_remain_time_bar_value->setTextVisible(true);

        progress_stage_remain_time_bar->addWidget(progress_stage_remain_time_bar_value);


        progress_bar_QV->addLayout(progress_stage_remain_time_bar);

        progress_own_robot_HP_bar = new QHBoxLayout();
        progress_own_robot_HP_bar->setSpacing(25);
        progress_own_robot_HP_bar->setObjectName(QStringLiteral("progress_own_robot_HP_bar"));
        progress_own_robot_HP_bar_title = new QLabel(verticalLayoutWidget);
        progress_own_robot_HP_bar_title->setObjectName(QStringLiteral("progress_own_robot_HP_bar_title"));
        progress_own_robot_HP_bar_title->setFont(font1);
        progress_own_robot_HP_bar_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_own_robot_HP_bar->addWidget(progress_own_robot_HP_bar_title);

        progress_own_robot_HP_bar_value = new QProgressBar(verticalLayoutWidget);
        progress_own_robot_HP_bar_value->setObjectName(QStringLiteral("progress_own_robot_HP_bar_value"));
        progress_own_robot_HP_bar_value->setFont(font2);
        progress_own_robot_HP_bar_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #00C18B,\n"
"        stop: 0.5 #00C18B,\n"
"        stop: 1 #00C18B\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_own_robot_HP_bar_value->setMaximum(600);
        progress_own_robot_HP_bar_value->setValue(555);
        progress_own_robot_HP_bar_value->setTextVisible(true);

        progress_own_robot_HP_bar->addWidget(progress_own_robot_HP_bar_value);


        progress_bar_QV->addLayout(progress_own_robot_HP_bar);

        progress_own_base_HP_bar = new QHBoxLayout();
        progress_own_base_HP_bar->setSpacing(25);
        progress_own_base_HP_bar->setObjectName(QStringLiteral("progress_own_base_HP_bar"));
        progress_own_base_HP_bar_title = new QLabel(verticalLayoutWidget);
        progress_own_base_HP_bar_title->setObjectName(QStringLiteral("progress_own_base_HP_bar_title"));
        progress_own_base_HP_bar_title->setFont(font1);
        progress_own_base_HP_bar_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_own_base_HP_bar->addWidget(progress_own_base_HP_bar_title);

        progress_own_base_HP_bar_value = new QProgressBar(verticalLayoutWidget);
        progress_own_base_HP_bar_value->setObjectName(QStringLiteral("progress_own_base_HP_bar_value"));
        progress_own_base_HP_bar_value->setFont(font2);
        progress_own_base_HP_bar_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #0F72EF,\n"
"        stop: 0.5 #0F72EF,\n"
"        stop: 1 #0F72EF\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_own_base_HP_bar_value->setMaximum(1600);
        progress_own_base_HP_bar_value->setValue(1400);
        progress_own_base_HP_bar_value->setTextVisible(true);

        progress_own_base_HP_bar->addWidget(progress_own_base_HP_bar_value);


        progress_bar_QV->addLayout(progress_own_base_HP_bar);

        progress_enemy_base_HP_bar = new QHBoxLayout();
        progress_enemy_base_HP_bar->setSpacing(25);
        progress_enemy_base_HP_bar->setObjectName(QStringLiteral("progress_enemy_base_HP_bar"));
        progress_enemy_base_HP_bar_title = new QLabel(verticalLayoutWidget);
        progress_enemy_base_HP_bar_title->setObjectName(QStringLiteral("progress_enemy_base_HP_bar_title"));
        progress_enemy_base_HP_bar_title->setFont(font1);
        progress_enemy_base_HP_bar_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_enemy_base_HP_bar->addWidget(progress_enemy_base_HP_bar_title);

        progress_enemy_base_HP_bar_value = new QProgressBar(verticalLayoutWidget);
        progress_enemy_base_HP_bar_value->setObjectName(QStringLiteral("progress_enemy_base_HP_bar_value"));
        progress_enemy_base_HP_bar_value->setFont(font2);
        progress_enemy_base_HP_bar_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #FF5169,\n"
"        stop: 0.5 #FF5169,\n"
"        stop: 1 #FF5169\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_enemy_base_HP_bar_value->setMaximum(1600);
        progress_enemy_base_HP_bar_value->setValue(900);
        progress_enemy_base_HP_bar_value->setTextVisible(true);

        progress_enemy_base_HP_bar->addWidget(progress_enemy_base_HP_bar_value);


        progress_bar_QV->addLayout(progress_enemy_base_HP_bar);

        progress_shoot_num_bar = new QHBoxLayout();
        progress_shoot_num_bar->setSpacing(25);
        progress_shoot_num_bar->setObjectName(QStringLiteral("progress_shoot_num_bar"));
        progress_shoot_num_bar_title = new QLabel(verticalLayoutWidget);
        progress_shoot_num_bar_title->setObjectName(QStringLiteral("progress_shoot_num_bar_title"));
        progress_shoot_num_bar_title->setFont(font1);
        progress_shoot_num_bar_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_shoot_num_bar->addWidget(progress_shoot_num_bar_title);

        progress_shoot_num_bar_value = new QProgressBar(verticalLayoutWidget);
        progress_shoot_num_bar_value->setObjectName(QStringLiteral("progress_shoot_num_bar_value"));
        progress_shoot_num_bar_value->setFont(font2);
        progress_shoot_num_bar_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #FED572,\n"
"        stop: 0.5 #FED572,\n"
"        stop: 1 #FED572\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_shoot_num_bar_value->setMaximum(600);
        progress_shoot_num_bar_value->setValue(80);
        progress_shoot_num_bar_value->setTextVisible(true);

        progress_shoot_num_bar->addWidget(progress_shoot_num_bar_value);


        progress_bar_QV->addLayout(progress_shoot_num_bar);

        progress_nav_state = new QHBoxLayout();
        progress_nav_state->setSpacing(25);
        progress_nav_state->setObjectName(QStringLiteral("progress_nav_state"));
        progress_nav_state_title = new QLabel(verticalLayoutWidget);
        progress_nav_state_title->setObjectName(QStringLiteral("progress_nav_state_title"));
        progress_nav_state_title->setFont(font1);
        progress_nav_state_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        progress_nav_state->addWidget(progress_nav_state_title);

        progress_nav_state_value = new QProgressBar(verticalLayoutWidget);
        progress_nav_state_value->setObjectName(QStringLiteral("progress_nav_state_value"));
        progress_nav_state_value->setFont(font2);
        progress_nav_state_value->setStyleSheet(QString::fromUtf8("QProgressBar {\n"
"    border: 1px solid #e6e9eb; /* \350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    border-radius: 10px;    /* \345\234\206\350\247\222 */\n"
"    background: #e6e9eb;       /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    text-align: center;     /* \346\226\207\345\255\227\345\261\205\344\270\255 */\n"
"    color: #ffffff;            /* \346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    height: 20px;           /* \351\253\230\345\272\246 */\n"
"}\n"
"\n"
"QProgressBar::chunk {\n"
"    background: qlineargradient(\n"
"        spread: repeat,\n"
"        x1: 0, y1: 0, x2: 1, y2: 1,\n"
"        stop: 0 #B58BE5,\n"
"        stop: 0.5 #B58BE5,\n"
"        stop: 1 #B58BE5\n"
"    );\n"
"    border-radius: 8px; /* \344\270\216\347\210\266\345\256\271\345\231\250\347\232\204\345\234\206\350\247\222\344\277\235\346\214\201\344\270\200\350\207\264 */\n"
"    margin: 1px;        /* \345\242\236\345\212\240\350\277\233\345\272\246\346\235\241\345\235\227\347\232\204\351"
                        "\227\264\350\267\235 */\n"
"}\n"
""));
        progress_nav_state_value->setMaximum(100);
        progress_nav_state_value->setValue(68);
        progress_nav_state_value->setTextVisible(true);

        progress_nav_state->addWidget(progress_nav_state_value);


        progress_bar_QV->addLayout(progress_nav_state);

        home = new QWidget(centralwidget);
        home->setObjectName(QStringLiteral("home"));
        home->setGeometry(QRect(0, 990, 1920, 100));
        home->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"\n"
""));
        horizontalLayoutWidget_12 = new QWidget(home);
        horizontalLayoutWidget_12->setObjectName(QStringLiteral("horizontalLayoutWidget_12"));
        horizontalLayoutWidget_12->setGeometry(QRect(-1, -1, 1921, 91));
        home_QH = new QHBoxLayout(horizontalLayoutWidget_12);
        home_QH->setObjectName(QStringLiteral("home_QH"));
        home_QH->setContentsMargins(0, 0, 0, 0);
        home_page = new QPushButton(horizontalLayoutWidget_12);
        home_page->setObjectName(QStringLiteral("home_page"));
        home_page->setAutoFillBackground(false);
        home_page->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"    background: none;      /* \351\232\220\350\227\217\350\203\214\346\231\257 */\n"
"}\n"
"\n"
""));
        QIcon icon;
        icon.addFile(QStringLiteral("resource/home.png"), QSize(), QIcon::Normal, QIcon::Off);
        home_page->setIcon(icon);
        home_page->setIconSize(QSize(40, 40));
        home_page->setAutoDefault(false);

        home_QH->addWidget(home_page);

        rules_page = new QPushButton(horizontalLayoutWidget_12);
        rules_page->setObjectName(QStringLiteral("rules_page"));
        rules_page->setAutoFillBackground(false);
        rules_page->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"    background: none;      /* \351\232\220\350\227\217\350\203\214\346\231\257 */\n"
"}\n"
"\n"
""));
        QIcon icon1;
        icon1.addFile(QStringLiteral("resource/rules.png"), QSize(), QIcon::Normal, QIcon::Off);
        rules_page->setIcon(icon1);
        rules_page->setIconSize(QSize(40, 40));
        rules_page->setAutoDefault(false);

        home_QH->addWidget(rules_page);

        debug_page = new QPushButton(horizontalLayoutWidget_12);
        debug_page->setObjectName(QStringLiteral("debug_page"));
        debug_page->setAutoFillBackground(false);
        debug_page->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"    background: none;      /* \351\232\220\350\227\217\350\203\214\346\231\257 */\n"
"}\n"
"\n"
""));
        QIcon icon2;
        icon2.addFile(QStringLiteral("resource/debug-16.png"), QSize(), QIcon::Normal, QIcon::Off);
        debug_page->setIcon(icon2);
        debug_page->setIconSize(QSize(40, 40));
        debug_page->setAutoDefault(false);

        home_QH->addWidget(debug_page);

        setting_page = new QPushButton(horizontalLayoutWidget_12);
        setting_page->setObjectName(QStringLiteral("setting_page"));
        setting_page->setAutoFillBackground(false);
        setting_page->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"    background: none;      /* \351\232\220\350\227\217\350\203\214\346\231\257 */\n"
"}\n"
"\n"
""));
        QIcon icon3;
        icon3.addFile(QStringLiteral("resource/cog.png"), QSize(), QIcon::Normal, QIcon::Off);
        setting_page->setIcon(icon3);
        setting_page->setIconSize(QSize(40, 40));
        setting_page->setAutoDefault(false);

        home_QH->addWidget(setting_page);

        monitor_board = new QWidget(centralwidget);
        monitor_board->setObjectName(QStringLiteral("monitor_board"));
        monitor_board->setGeometry(QRect(20, 20, 801, 311));
        sizePolicy.setHeightForWidth(monitor_board->sizePolicy().hasHeightForWidth());
        monitor_board->setSizePolicy(sizePolicy);
        monitor_board->setLayoutDirection(Qt::LeftToRight);
        monitor_board->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        horizontalLayoutWidget_7 = new QWidget(monitor_board);
        horizontalLayoutWidget_7->setObjectName(QStringLiteral("horizontalLayoutWidget_7"));
        horizontalLayoutWidget_7->setGeometry(QRect(20, 160, 751, 111));
        monitor_board_2 = new QHBoxLayout(horizontalLayoutWidget_7);
        monitor_board_2->setSpacing(20);
        monitor_board_2->setObjectName(QStringLiteral("monitor_board_2"));
        monitor_board_2->setContentsMargins(5, 5, 5, 5);
        monitor_task_id_QW = new QWidget(horizontalLayoutWidget_7);
        monitor_task_id_QW->setObjectName(QStringLiteral("monitor_task_id_QW"));
        monitor_task_id_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_9 = new QWidget(monitor_task_id_QW);
        verticalLayoutWidget_9->setObjectName(QStringLiteral("verticalLayoutWidget_9"));
        verticalLayoutWidget_9->setGeometry(QRect(0, 0, 161, 91));
        monitor_task_id_QV = new QVBoxLayout(verticalLayoutWidget_9);
        monitor_task_id_QV->setObjectName(QStringLiteral("monitor_task_id_QV"));
        monitor_task_id_QV->setContentsMargins(15, 15, 15, 15);
        monitor_task_id_title = new QLabel(verticalLayoutWidget_9);
        monitor_task_id_title->setObjectName(QStringLiteral("monitor_task_id_title"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(monitor_task_id_title->sizePolicy().hasHeightForWidth());
        monitor_task_id_title->setSizePolicy(sizePolicy1);
        QFont font3;
        font3.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font3.setPointSize(9);
        monitor_task_id_title->setFont(font3);
        monitor_task_id_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_task_id_QV->addWidget(monitor_task_id_title);

        monitor_task_id_state = new QLabel(verticalLayoutWidget_9);
        monitor_task_id_state->setObjectName(QStringLiteral("monitor_task_id_state"));
        sizePolicy1.setHeightForWidth(monitor_task_id_state->sizePolicy().hasHeightForWidth());
        monitor_task_id_state->setSizePolicy(sizePolicy1);
        QFont font4;
        font4.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font4.setPointSize(14);
        font4.setUnderline(false);
        monitor_task_id_state->setFont(font4);
        monitor_task_id_state->setStyleSheet(QStringLiteral("color: rgb(15, 114, 239);"));

        monitor_task_id_QV->addWidget(monitor_task_id_state);


        monitor_board_2->addWidget(monitor_task_id_QW);

        monitor_goal_pose_QW = new QWidget(horizontalLayoutWidget_7);
        monitor_goal_pose_QW->setObjectName(QStringLiteral("monitor_goal_pose_QW"));
        monitor_goal_pose_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_8 = new QWidget(monitor_goal_pose_QW);
        verticalLayoutWidget_8->setObjectName(QStringLiteral("verticalLayoutWidget_8"));
        verticalLayoutWidget_8->setGeometry(QRect(0, 0, 161, 91));
        monitor_goal_pose_QV = new QVBoxLayout(verticalLayoutWidget_8);
        monitor_goal_pose_QV->setObjectName(QStringLiteral("monitor_goal_pose_QV"));
        monitor_goal_pose_QV->setContentsMargins(15, 15, 15, 15);
        monitor_goal_pose_title = new QLabel(verticalLayoutWidget_8);
        monitor_goal_pose_title->setObjectName(QStringLiteral("monitor_goal_pose_title"));
        sizePolicy1.setHeightForWidth(monitor_goal_pose_title->sizePolicy().hasHeightForWidth());
        monitor_goal_pose_title->setSizePolicy(sizePolicy1);
        monitor_goal_pose_title->setFont(font3);
        monitor_goal_pose_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_goal_pose_QV->addWidget(monitor_goal_pose_title);

        monitor_goal_pose_state = new QLabel(verticalLayoutWidget_8);
        monitor_goal_pose_state->setObjectName(QStringLiteral("monitor_goal_pose_state"));
        sizePolicy1.setHeightForWidth(monitor_goal_pose_state->sizePolicy().hasHeightForWidth());
        monitor_goal_pose_state->setSizePolicy(sizePolicy1);
        monitor_goal_pose_state->setFont(font4);
        monitor_goal_pose_state->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        monitor_goal_pose_QV->addWidget(monitor_goal_pose_state);


        monitor_board_2->addWidget(monitor_goal_pose_QW);

        monitor_vision_status_QW = new QWidget(horizontalLayoutWidget_7);
        monitor_vision_status_QW->setObjectName(QStringLiteral("monitor_vision_status_QW"));
        monitor_vision_status_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_3 = new QWidget(monitor_vision_status_QW);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(0, 0, 161, 91));
        monitor_vision_status_QV = new QVBoxLayout(verticalLayoutWidget_3);
        monitor_vision_status_QV->setObjectName(QStringLiteral("monitor_vision_status_QV"));
        monitor_vision_status_QV->setContentsMargins(15, 15, 15, 15);
        monitor_vision_status_title = new QLabel(verticalLayoutWidget_3);
        monitor_vision_status_title->setObjectName(QStringLiteral("monitor_vision_status_title"));
        sizePolicy1.setHeightForWidth(monitor_vision_status_title->sizePolicy().hasHeightForWidth());
        monitor_vision_status_title->setSizePolicy(sizePolicy1);
        monitor_vision_status_title->setFont(font3);
        monitor_vision_status_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_vision_status_QV->addWidget(monitor_vision_status_title);

        monitor_vision_status_sate = new QLabel(verticalLayoutWidget_3);
        monitor_vision_status_sate->setObjectName(QStringLiteral("monitor_vision_status_sate"));
        sizePolicy1.setHeightForWidth(monitor_vision_status_sate->sizePolicy().hasHeightForWidth());
        monitor_vision_status_sate->setSizePolicy(sizePolicy1);
        monitor_vision_status_sate->setFont(font4);
        monitor_vision_status_sate->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        monitor_vision_status_QV->addWidget(monitor_vision_status_sate);


        monitor_board_2->addWidget(monitor_vision_status_QW);

        monitor_enemy_pose_QW = new QWidget(horizontalLayoutWidget_7);
        monitor_enemy_pose_QW->setObjectName(QStringLiteral("monitor_enemy_pose_QW"));
        monitor_enemy_pose_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_10 = new QWidget(monitor_enemy_pose_QW);
        verticalLayoutWidget_10->setObjectName(QStringLiteral("verticalLayoutWidget_10"));
        verticalLayoutWidget_10->setGeometry(QRect(0, 0, 161, 91));
        monitor_enemy_pose_QV = new QVBoxLayout(verticalLayoutWidget_10);
        monitor_enemy_pose_QV->setObjectName(QStringLiteral("monitor_enemy_pose_QV"));
        monitor_enemy_pose_QV->setContentsMargins(15, 15, 15, 15);
        monitor_enemy_pose_title = new QLabel(verticalLayoutWidget_10);
        monitor_enemy_pose_title->setObjectName(QStringLiteral("monitor_enemy_pose_title"));
        sizePolicy1.setHeightForWidth(monitor_enemy_pose_title->sizePolicy().hasHeightForWidth());
        monitor_enemy_pose_title->setSizePolicy(sizePolicy1);
        monitor_enemy_pose_title->setFont(font3);
        monitor_enemy_pose_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_enemy_pose_QV->addWidget(monitor_enemy_pose_title);

        monitor_enemy_pose_state = new QLabel(verticalLayoutWidget_10);
        monitor_enemy_pose_state->setObjectName(QStringLiteral("monitor_enemy_pose_state"));
        sizePolicy1.setHeightForWidth(monitor_enemy_pose_state->sizePolicy().hasHeightForWidth());
        monitor_enemy_pose_state->setSizePolicy(sizePolicy1);
        monitor_enemy_pose_state->setFont(font4);
        monitor_enemy_pose_state->setStyleSheet(QStringLiteral("color: rgb(15, 114, 239);"));

        monitor_enemy_pose_QV->addWidget(monitor_enemy_pose_state);


        monitor_board_2->addWidget(monitor_enemy_pose_QW);

        horizontalLayoutWidget = new QWidget(monitor_board);
        horizontalLayoutWidget->setObjectName(QStringLiteral("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(19, 30, 751, 111));
        monitor_board_1 = new QHBoxLayout(horizontalLayoutWidget);
        monitor_board_1->setSpacing(20);
        monitor_board_1->setObjectName(QStringLiteral("monitor_board_1"));
        monitor_board_1->setContentsMargins(5, 5, 5, 5);
        monitor_game_progress_QW = new QWidget(horizontalLayoutWidget);
        monitor_game_progress_QW->setObjectName(QStringLiteral("monitor_game_progress_QW"));
        monitor_game_progress_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_5 = new QWidget(monitor_game_progress_QW);
        verticalLayoutWidget_5->setObjectName(QStringLiteral("verticalLayoutWidget_5"));
        verticalLayoutWidget_5->setGeometry(QRect(0, 0, 161, 91));
        monitor_game_progress_QV = new QVBoxLayout(verticalLayoutWidget_5);
        monitor_game_progress_QV->setObjectName(QStringLiteral("monitor_game_progress_QV"));
        monitor_game_progress_QV->setContentsMargins(15, 15, 15, 15);
        monitor_game_progress_title = new QLabel(verticalLayoutWidget_5);
        monitor_game_progress_title->setObjectName(QStringLiteral("monitor_game_progress_title"));
        sizePolicy1.setHeightForWidth(monitor_game_progress_title->sizePolicy().hasHeightForWidth());
        monitor_game_progress_title->setSizePolicy(sizePolicy1);
        monitor_game_progress_title->setFont(font3);
        monitor_game_progress_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_game_progress_QV->addWidget(monitor_game_progress_title);

        monitor_game_progress_state = new QLabel(verticalLayoutWidget_5);
        monitor_game_progress_state->setObjectName(QStringLiteral("monitor_game_progress_state"));
        sizePolicy1.setHeightForWidth(monitor_game_progress_state->sizePolicy().hasHeightForWidth());
        monitor_game_progress_state->setSizePolicy(sizePolicy1);
        monitor_game_progress_state->setFont(font4);
        monitor_game_progress_state->setStyleSheet(QStringLiteral("color: rgb(0, 193, 139);"));

        monitor_game_progress_QV->addWidget(monitor_game_progress_state);


        monitor_board_1->addWidget(monitor_game_progress_QW);

        monitor_HP_deduction_reason_QW = new QWidget(horizontalLayoutWidget);
        monitor_HP_deduction_reason_QW->setObjectName(QStringLiteral("monitor_HP_deduction_reason_QW"));
        monitor_HP_deduction_reason_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_4 = new QWidget(monitor_HP_deduction_reason_QW);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(0, 0, 161, 91));
        monitor_HP_deduction_reason_QV = new QVBoxLayout(verticalLayoutWidget_4);
        monitor_HP_deduction_reason_QV->setObjectName(QStringLiteral("monitor_HP_deduction_reason_QV"));
        monitor_HP_deduction_reason_QV->setContentsMargins(15, 15, 15, 15);
        monitor_HP_deduction_reason_title = new QLabel(verticalLayoutWidget_4);
        monitor_HP_deduction_reason_title->setObjectName(QStringLiteral("monitor_HP_deduction_reason_title"));
        sizePolicy1.setHeightForWidth(monitor_HP_deduction_reason_title->sizePolicy().hasHeightForWidth());
        monitor_HP_deduction_reason_title->setSizePolicy(sizePolicy1);
        monitor_HP_deduction_reason_title->setFont(font3);
        monitor_HP_deduction_reason_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_HP_deduction_reason_QV->addWidget(monitor_HP_deduction_reason_title);

        monitor_HP_deduction_reason_state = new QLabel(verticalLayoutWidget_4);
        monitor_HP_deduction_reason_state->setObjectName(QStringLiteral("monitor_HP_deduction_reason_state"));
        sizePolicy1.setHeightForWidth(monitor_HP_deduction_reason_state->sizePolicy().hasHeightForWidth());
        monitor_HP_deduction_reason_state->setSizePolicy(sizePolicy1);
        monitor_HP_deduction_reason_state->setFont(font4);
        monitor_HP_deduction_reason_state->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        monitor_HP_deduction_reason_QV->addWidget(monitor_HP_deduction_reason_state);


        monitor_board_1->addWidget(monitor_HP_deduction_reason_QW);

        monitor_armor_id_QW = new QWidget(horizontalLayoutWidget);
        monitor_armor_id_QW->setObjectName(QStringLiteral("monitor_armor_id_QW"));
        monitor_armor_id_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_6 = new QWidget(monitor_armor_id_QW);
        verticalLayoutWidget_6->setObjectName(QStringLiteral("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(0, 0, 161, 91));
        monitor_armor_id_QV = new QVBoxLayout(verticalLayoutWidget_6);
        monitor_armor_id_QV->setObjectName(QStringLiteral("monitor_armor_id_QV"));
        monitor_armor_id_QV->setContentsMargins(15, 15, 15, 15);
        monitor_armor_id_title = new QLabel(verticalLayoutWidget_6);
        monitor_armor_id_title->setObjectName(QStringLiteral("monitor_armor_id_title"));
        sizePolicy1.setHeightForWidth(monitor_armor_id_title->sizePolicy().hasHeightForWidth());
        monitor_armor_id_title->setSizePolicy(sizePolicy1);
        monitor_armor_id_title->setFont(font3);
        monitor_armor_id_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_armor_id_QV->addWidget(monitor_armor_id_title);

        monitor_armor_id_state = new QLabel(verticalLayoutWidget_6);
        monitor_armor_id_state->setObjectName(QStringLiteral("monitor_armor_id_state"));
        sizePolicy1.setHeightForWidth(monitor_armor_id_state->sizePolicy().hasHeightForWidth());
        monitor_armor_id_state->setSizePolicy(sizePolicy1);
        monitor_armor_id_state->setFont(font4);
        monitor_armor_id_state->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        monitor_armor_id_QV->addWidget(monitor_armor_id_state);


        monitor_board_1->addWidget(monitor_armor_id_QW);

        monitor_event_data_QW = new QWidget(horizontalLayoutWidget);
        monitor_event_data_QW->setObjectName(QStringLiteral("monitor_event_data_QW"));
        monitor_event_data_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_7 = new QWidget(monitor_event_data_QW);
        verticalLayoutWidget_7->setObjectName(QStringLiteral("verticalLayoutWidget_7"));
        verticalLayoutWidget_7->setGeometry(QRect(0, 0, 161, 92));
        monitor_event_data_QV = new QVBoxLayout(verticalLayoutWidget_7);
        monitor_event_data_QV->setObjectName(QStringLiteral("monitor_event_data_QV"));
        monitor_event_data_QV->setContentsMargins(15, 15, 15, 15);
        monitor_event_data_title = new QLabel(verticalLayoutWidget_7);
        monitor_event_data_title->setObjectName(QStringLiteral("monitor_event_data_title"));
        sizePolicy1.setHeightForWidth(monitor_event_data_title->sizePolicy().hasHeightForWidth());
        monitor_event_data_title->setSizePolicy(sizePolicy1);
        monitor_event_data_title->setFont(font3);
        monitor_event_data_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        monitor_event_data_QV->addWidget(monitor_event_data_title);

        monitor_event_data_state = new QLabel(verticalLayoutWidget_7);
        monitor_event_data_state->setObjectName(QStringLiteral("monitor_event_data_state"));
        sizePolicy1.setHeightForWidth(monitor_event_data_state->sizePolicy().hasHeightForWidth());
        monitor_event_data_state->setSizePolicy(sizePolicy1);
        monitor_event_data_state->setFont(font4);
        monitor_event_data_state->setStyleSheet(QStringLiteral("color: rgb(153, 153, 153);"));

        monitor_event_data_QV->addWidget(monitor_event_data_state);


        monitor_board_1->addWidget(monitor_event_data_QW);

        logQS = new QScrollArea(centralwidget);
        logQS->setObjectName(QStringLiteral("logQS"));
        logQS->setGeometry(QRect(20, 700, 801, 271));
        logQS->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        logQS->setWidgetResizable(true);
        logQS_QW = new QWidget();
        logQS_QW->setObjectName(QStringLiteral("logQS_QW"));
        logQS_QW->setGeometry(QRect(0, 0, 801, 271));
        logQS_LOG_TEXT = new QTextBrowser(logQS_QW);
        logQS_LOG_TEXT->setObjectName(QStringLiteral("logQS_LOG_TEXT"));
        logQS_LOG_TEXT->setGeometry(QRect(10, 10, 781, 251));
        QFont font5;
        font5.setFamily(QString::fromUtf8("\351\224\220\345\255\227\344\272\221\345\255\227\345\272\223\347\262\227\345\234\206GBK"));
        logQS_LOG_TEXT->setFont(font5);
        logQS_LOG_TEXT->setStyleSheet(QString::fromUtf8("QTextBrowser {\n"
"    border: 30px solid transparent; /* \345\220\221\345\244\226\346\211\251\345\261\225 30 \345\203\217\347\264\240 */\n"
"    padding: 0px; /* \347\247\273\351\231\244\345\206\205\350\276\271\350\267\235 */\n"
"    border-radius: 25px; /* \345\234\206\350\247\222 25 \345\203\217\347\264\240 */\n"
"    border: 10px solid rgb(240, 240, 240); /* \350\256\276\347\275\256\350\276\271\346\241\206\351\242\234\350\211\262\345\222\214\345\216\232\345\272\246 */\n"
"    background-clip: padding-box; /* \351\230\262\346\255\242\350\203\214\346\231\257\350\266\205\345\207\272\345\234\206\350\247\222\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\256\276\347\275\256\350\203\214\346\231\257\351\242\234\350\211\262\357\274\214\351\230\262\346\255\242\351\200\217\346\230\216 */\n"
"	color: rgb(145, 146, 152);\n"
"}\n"
"\n"
"/* \345\236\202\347\233\264\346\273\232\345\212\250\346\235\241 */\n"
"QScrollBar:vertical {\n"
"    background: #f5f5f5; /* \346\273\232\345\212\250\346"
                        "\235\241\350\203\214\346\231\257 */\n"
"    width: 12px; /* \346\273\232\345\212\250\346\235\241\345\256\275\345\272\246 */\n"
"    margin: 0px 0px 0px 0px; /* \344\270\212\344\270\213\343\200\201\345\267\246\345\217\263\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"    border: none; /* \345\216\273\351\231\244\346\273\232\345\212\250\346\235\241\350\276\271\346\241\206 */\n"
"}\n"
"\n"
"/* \346\273\221\345\235\227 */\n"
"QScrollBar::handle:vertical {\n"
"    background: #7FB0FF; /* \346\273\221\345\235\227\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    min-height: 20px; /* \346\273\221\345\235\227\347\232\204\346\234\200\345\260\217\351\253\230\345\272\246 */\n"
"    border-radius: 6px; /* \346\273\221\345\235\227\347\232\204\345\234\206\350\247\222 */\n"
"    border: none; /* \351\230\262\346\255\242\351\200\217\346\230\216\350\276\271\346\241\206\345\271\262\346\211\260 */\n"
"}\n"
"\n"
"/* \351\274\240\346\240\207\346\202\254\345\201\234\345\234\250\346\273\221\345\235\227\344\270\212\347"
                        "\232\204\346\225\210\346\236\234 */\n"
"QScrollBar::handle:vertical:hover {\n"
"    background: #0F72EF; /* \346\273\221\345\235\227\346\202\254\345\201\234\346\227\266\347\232\204\351\242\234\350\211\262 */\n"
"}\n"
"\n"
"/* \347\247\273\351\231\244\344\270\212\344\270\213\347\256\255\345\244\264 */\n"
"QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {\n"
"    background: none;\n"
"    border: none;\n"
"    height: 0px;\n"
"}\n"
"\n"
"/* \346\273\232\345\212\250\346\235\241\346\247\275 */\n"
"QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {\n"
"    background: rgb(230, 233, 235); /* \346\273\232\345\212\250\346\235\241\346\247\275\350\203\214\346\231\257 */\n"
"    border-radius: 6px; /* \346\247\275\347\232\204\345\234\206\350\247\222 */\n"
"}\n"
"\n"
""));
        logQS->setWidget(logQS_QW);
        Sim_Data_QW = new QWidget(centralwidget);
        Sim_Data_QW->setObjectName(QStringLiteral("Sim_Data_QW"));
        Sim_Data_QW->setGeometry(QRect(1570, 20, 321, 691));
        sizePolicy.setHeightForWidth(Sim_Data_QW->sizePolicy().hasHeightForWidth());
        Sim_Data_QW->setSizePolicy(sizePolicy);
        Sim_Data_QW->setLayoutDirection(Qt::LeftToRight);
        Sim_Data_QW->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        verticalLayoutWidget_2 = new QWidget(Sim_Data_QW);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(20, 140, 131, 471));
        Sim_Data_QV_1 = new QVBoxLayout(verticalLayoutWidget_2);
        Sim_Data_QV_1->setObjectName(QStringLiteral("Sim_Data_QV_1"));
        Sim_Data_QV_1->setContentsMargins(0, 0, 0, 0);
        Sim_Data_game_progress_QW = new QWidget(verticalLayoutWidget_2);
        Sim_Data_game_progress_QW->setObjectName(QStringLiteral("Sim_Data_game_progress_QW"));
        Sim_Data_game_progress_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_16 = new QWidget(Sim_Data_game_progress_QW);
        verticalLayoutWidget_16->setObjectName(QStringLiteral("verticalLayoutWidget_16"));
        verticalLayoutWidget_16->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_game_progress_QV = new QVBoxLayout(verticalLayoutWidget_16);
        Sim_Data_game_progress_QV->setObjectName(QStringLiteral("Sim_Data_game_progress_QV"));
        Sim_Data_game_progress_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_game_progress_title = new QLabel(verticalLayoutWidget_16);
        Sim_Data_game_progress_title->setObjectName(QStringLiteral("Sim_Data_game_progress_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_game_progress_title->sizePolicy().hasHeightForWidth());
        Sim_Data_game_progress_title->setSizePolicy(sizePolicy1);
        QFont font6;
        font6.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font6.setPointSize(9);
        Sim_Data_game_progress_title->setFont(font6);
        Sim_Data_game_progress_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_game_progress_QV->addWidget(Sim_Data_game_progress_title);

        Sim_Data_game_progress_QL = new QLineEdit(verticalLayoutWidget_16);
        Sim_Data_game_progress_QL->setObjectName(QStringLiteral("Sim_Data_game_progress_QL"));
        QFont font7;
        font7.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font7.setPointSize(14);
        font7.setBold(false);
        font7.setWeight(50);
        Sim_Data_game_progress_QL->setFont(font7);
        Sim_Data_game_progress_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_game_progress_QV->addWidget(Sim_Data_game_progress_QL);


        Sim_Data_QV_1->addWidget(Sim_Data_game_progress_QW);

        Sim_Data_own_robot_HP_QW = new QWidget(verticalLayoutWidget_2);
        Sim_Data_own_robot_HP_QW->setObjectName(QStringLiteral("Sim_Data_own_robot_HP_QW"));
        Sim_Data_own_robot_HP_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_18 = new QWidget(Sim_Data_own_robot_HP_QW);
        verticalLayoutWidget_18->setObjectName(QStringLiteral("verticalLayoutWidget_18"));
        verticalLayoutWidget_18->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_own_robot_HP_QV = new QVBoxLayout(verticalLayoutWidget_18);
        Sim_Data_own_robot_HP_QV->setObjectName(QStringLiteral("Sim_Data_own_robot_HP_QV"));
        Sim_Data_own_robot_HP_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_own_robot_HP_title = new QLabel(verticalLayoutWidget_18);
        Sim_Data_own_robot_HP_title->setObjectName(QStringLiteral("Sim_Data_own_robot_HP_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_own_robot_HP_title->sizePolicy().hasHeightForWidth());
        Sim_Data_own_robot_HP_title->setSizePolicy(sizePolicy1);
        Sim_Data_own_robot_HP_title->setFont(font6);
        Sim_Data_own_robot_HP_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_own_robot_HP_QV->addWidget(Sim_Data_own_robot_HP_title);

        Sim_Data_own_robot_HP_QL = new QLineEdit(verticalLayoutWidget_18);
        Sim_Data_own_robot_HP_QL->setObjectName(QStringLiteral("Sim_Data_own_robot_HP_QL"));
        Sim_Data_own_robot_HP_QL->setFont(font7);
        Sim_Data_own_robot_HP_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_own_robot_HP_QV->addWidget(Sim_Data_own_robot_HP_QL);


        Sim_Data_QV_1->addWidget(Sim_Data_own_robot_HP_QW);

        Sim_Data_enemy_base_HP_QW = new QWidget(verticalLayoutWidget_2);
        Sim_Data_enemy_base_HP_QW->setObjectName(QStringLiteral("Sim_Data_enemy_base_HP_QW"));
        Sim_Data_enemy_base_HP_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_15 = new QWidget(Sim_Data_enemy_base_HP_QW);
        verticalLayoutWidget_15->setObjectName(QStringLiteral("verticalLayoutWidget_15"));
        verticalLayoutWidget_15->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_enemy_base_HP_QV = new QVBoxLayout(verticalLayoutWidget_15);
        Sim_Data_enemy_base_HP_QV->setObjectName(QStringLiteral("Sim_Data_enemy_base_HP_QV"));
        Sim_Data_enemy_base_HP_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_enemy_base_HP_title = new QLabel(verticalLayoutWidget_15);
        Sim_Data_enemy_base_HP_title->setObjectName(QStringLiteral("Sim_Data_enemy_base_HP_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_enemy_base_HP_title->sizePolicy().hasHeightForWidth());
        Sim_Data_enemy_base_HP_title->setSizePolicy(sizePolicy1);
        Sim_Data_enemy_base_HP_title->setFont(font6);
        Sim_Data_enemy_base_HP_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_enemy_base_HP_QV->addWidget(Sim_Data_enemy_base_HP_title);

        Sim_Data_enemy_base_HP_QL = new QLineEdit(verticalLayoutWidget_15);
        Sim_Data_enemy_base_HP_QL->setObjectName(QStringLiteral("Sim_Data_enemy_base_HP_QL"));
        Sim_Data_enemy_base_HP_QL->setFont(font7);
        Sim_Data_enemy_base_HP_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_enemy_base_HP_QV->addWidget(Sim_Data_enemy_base_HP_QL);


        Sim_Data_QV_1->addWidget(Sim_Data_enemy_base_HP_QW);

        Sim_Data_HP_deduction_reason_QW = new QWidget(verticalLayoutWidget_2);
        Sim_Data_HP_deduction_reason_QW->setObjectName(QStringLiteral("Sim_Data_HP_deduction_reason_QW"));
        Sim_Data_HP_deduction_reason_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_23 = new QWidget(Sim_Data_HP_deduction_reason_QW);
        verticalLayoutWidget_23->setObjectName(QStringLiteral("verticalLayoutWidget_23"));
        verticalLayoutWidget_23->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_HP_deduction_reason_QV = new QVBoxLayout(verticalLayoutWidget_23);
        Sim_Data_HP_deduction_reason_QV->setObjectName(QStringLiteral("Sim_Data_HP_deduction_reason_QV"));
        Sim_Data_HP_deduction_reason_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_HP_deduction_reason_title = new QLabel(verticalLayoutWidget_23);
        Sim_Data_HP_deduction_reason_title->setObjectName(QStringLiteral("Sim_Data_HP_deduction_reason_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_HP_deduction_reason_title->sizePolicy().hasHeightForWidth());
        Sim_Data_HP_deduction_reason_title->setSizePolicy(sizePolicy1);
        Sim_Data_HP_deduction_reason_title->setFont(font6);
        Sim_Data_HP_deduction_reason_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_HP_deduction_reason_QV->addWidget(Sim_Data_HP_deduction_reason_title);

        Sim_Data_HP_deduction_reason_QL = new QLineEdit(verticalLayoutWidget_23);
        Sim_Data_HP_deduction_reason_QL->setObjectName(QStringLiteral("Sim_Data_HP_deduction_reason_QL"));
        Sim_Data_HP_deduction_reason_QL->setFont(font7);
        Sim_Data_HP_deduction_reason_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_HP_deduction_reason_QV->addWidget(Sim_Data_HP_deduction_reason_QL);


        Sim_Data_QV_1->addWidget(Sim_Data_HP_deduction_reason_QW);

        Sim_Data_shoot_num_QW = new QWidget(verticalLayoutWidget_2);
        Sim_Data_shoot_num_QW->setObjectName(QStringLiteral("Sim_Data_shoot_num_QW"));
        Sim_Data_shoot_num_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_21 = new QWidget(Sim_Data_shoot_num_QW);
        verticalLayoutWidget_21->setObjectName(QStringLiteral("verticalLayoutWidget_21"));
        verticalLayoutWidget_21->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_shoot_num_QV = new QVBoxLayout(verticalLayoutWidget_21);
        Sim_Data_shoot_num_QV->setObjectName(QStringLiteral("Sim_Data_shoot_num_QV"));
        Sim_Data_shoot_num_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_shoot_num_title = new QLabel(verticalLayoutWidget_21);
        Sim_Data_shoot_num_title->setObjectName(QStringLiteral("Sim_Data_shoot_num_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_shoot_num_title->sizePolicy().hasHeightForWidth());
        Sim_Data_shoot_num_title->setSizePolicy(sizePolicy1);
        Sim_Data_shoot_num_title->setFont(font6);
        Sim_Data_shoot_num_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_shoot_num_QV->addWidget(Sim_Data_shoot_num_title);

        Sim_Data_shoot_num_QL = new QLineEdit(verticalLayoutWidget_21);
        Sim_Data_shoot_num_QL->setObjectName(QStringLiteral("Sim_Data_shoot_num_QL"));
        Sim_Data_shoot_num_QL->setFont(font7);
        Sim_Data_shoot_num_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_shoot_num_QV->addWidget(Sim_Data_shoot_num_QL);


        Sim_Data_QV_1->addWidget(Sim_Data_shoot_num_QW);

        verticalLayoutWidget_11 = new QWidget(Sim_Data_QW);
        verticalLayoutWidget_11->setObjectName(QStringLiteral("verticalLayoutWidget_11"));
        verticalLayoutWidget_11->setGeometry(QRect(170, 140, 131, 471));
        Sim_Data_QV_2 = new QVBoxLayout(verticalLayoutWidget_11);
        Sim_Data_QV_2->setObjectName(QStringLiteral("Sim_Data_QV_2"));
        Sim_Data_QV_2->setContentsMargins(0, 0, 0, 0);
        Sim_Data_stage_remain_time_QW = new QWidget(verticalLayoutWidget_11);
        Sim_Data_stage_remain_time_QW->setObjectName(QStringLiteral("Sim_Data_stage_remain_time_QW"));
        Sim_Data_stage_remain_time_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_17 = new QWidget(Sim_Data_stage_remain_time_QW);
        verticalLayoutWidget_17->setObjectName(QStringLiteral("verticalLayoutWidget_17"));
        verticalLayoutWidget_17->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_stage_remain_time_QV = new QVBoxLayout(verticalLayoutWidget_17);
        Sim_Data_stage_remain_time_QV->setObjectName(QStringLiteral("Sim_Data_stage_remain_time_QV"));
        Sim_Data_stage_remain_time_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_stage_remain_time_title = new QLabel(verticalLayoutWidget_17);
        Sim_Data_stage_remain_time_title->setObjectName(QStringLiteral("Sim_Data_stage_remain_time_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_stage_remain_time_title->sizePolicy().hasHeightForWidth());
        Sim_Data_stage_remain_time_title->setSizePolicy(sizePolicy1);
        Sim_Data_stage_remain_time_title->setFont(font6);
        Sim_Data_stage_remain_time_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_stage_remain_time_QV->addWidget(Sim_Data_stage_remain_time_title);

        Sim_Data_stage_remain_time_QL = new QLineEdit(verticalLayoutWidget_17);
        Sim_Data_stage_remain_time_QL->setObjectName(QStringLiteral("Sim_Data_stage_remain_time_QL"));
        Sim_Data_stage_remain_time_QL->setFont(font7);
        Sim_Data_stage_remain_time_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_stage_remain_time_QV->addWidget(Sim_Data_stage_remain_time_QL);


        Sim_Data_QV_2->addWidget(Sim_Data_stage_remain_time_QW);

        Sim_Data_vision_status_QW = new QWidget(verticalLayoutWidget_11);
        Sim_Data_vision_status_QW->setObjectName(QStringLiteral("Sim_Data_vision_status_QW"));
        Sim_Data_vision_status_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_22 = new QWidget(Sim_Data_vision_status_QW);
        verticalLayoutWidget_22->setObjectName(QStringLiteral("verticalLayoutWidget_22"));
        verticalLayoutWidget_22->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_vision_status_QV = new QVBoxLayout(verticalLayoutWidget_22);
        Sim_Data_vision_status_QV->setObjectName(QStringLiteral("Sim_Data_vision_status_QV"));
        Sim_Data_vision_status_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_vision_status_title = new QLabel(verticalLayoutWidget_22);
        Sim_Data_vision_status_title->setObjectName(QStringLiteral("Sim_Data_vision_status_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_vision_status_title->sizePolicy().hasHeightForWidth());
        Sim_Data_vision_status_title->setSizePolicy(sizePolicy1);
        Sim_Data_vision_status_title->setFont(font6);
        Sim_Data_vision_status_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_vision_status_QV->addWidget(Sim_Data_vision_status_title);

        Sim_Data_vision_status_QL = new QLineEdit(verticalLayoutWidget_22);
        Sim_Data_vision_status_QL->setObjectName(QStringLiteral("Sim_Data_vision_status_QL"));
        Sim_Data_vision_status_QL->setFont(font7);
        Sim_Data_vision_status_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_vision_status_QV->addWidget(Sim_Data_vision_status_QL);


        Sim_Data_QV_2->addWidget(Sim_Data_vision_status_QW);

        Sim_Data_own_base_HP_QW = new QWidget(verticalLayoutWidget_11);
        Sim_Data_own_base_HP_QW->setObjectName(QStringLiteral("Sim_Data_own_base_HP_QW"));
        Sim_Data_own_base_HP_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_19 = new QWidget(Sim_Data_own_base_HP_QW);
        verticalLayoutWidget_19->setObjectName(QStringLiteral("verticalLayoutWidget_19"));
        verticalLayoutWidget_19->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_own_base_HP_QV = new QVBoxLayout(verticalLayoutWidget_19);
        Sim_Data_own_base_HP_QV->setObjectName(QStringLiteral("Sim_Data_own_base_HP_QV"));
        Sim_Data_own_base_HP_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_own_base_HP_title = new QLabel(verticalLayoutWidget_19);
        Sim_Data_own_base_HP_title->setObjectName(QStringLiteral("Sim_Data_own_base_HP_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_own_base_HP_title->sizePolicy().hasHeightForWidth());
        Sim_Data_own_base_HP_title->setSizePolicy(sizePolicy1);
        Sim_Data_own_base_HP_title->setFont(font6);
        Sim_Data_own_base_HP_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_own_base_HP_QV->addWidget(Sim_Data_own_base_HP_title);

        Sim_Data_own_base_HP_QL = new QLineEdit(verticalLayoutWidget_19);
        Sim_Data_own_base_HP_QL->setObjectName(QStringLiteral("Sim_Data_own_base_HP_QL"));
        Sim_Data_own_base_HP_QL->setFont(font7);
        Sim_Data_own_base_HP_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_own_base_HP_QV->addWidget(Sim_Data_own_base_HP_QL);


        Sim_Data_QV_2->addWidget(Sim_Data_own_base_HP_QW);

        Sim_Data_event_data_QW = new QWidget(verticalLayoutWidget_11);
        Sim_Data_event_data_QW->setObjectName(QStringLiteral("Sim_Data_event_data_QW"));
        Sim_Data_event_data_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_20 = new QWidget(Sim_Data_event_data_QW);
        verticalLayoutWidget_20->setObjectName(QStringLiteral("verticalLayoutWidget_20"));
        verticalLayoutWidget_20->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_event_data_QV = new QVBoxLayout(verticalLayoutWidget_20);
        Sim_Data_event_data_QV->setObjectName(QStringLiteral("Sim_Data_event_data_QV"));
        Sim_Data_event_data_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_event_data_title = new QLabel(verticalLayoutWidget_20);
        Sim_Data_event_data_title->setObjectName(QStringLiteral("Sim_Data_event_data_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_event_data_title->sizePolicy().hasHeightForWidth());
        Sim_Data_event_data_title->setSizePolicy(sizePolicy1);
        Sim_Data_event_data_title->setFont(font6);
        Sim_Data_event_data_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_event_data_QV->addWidget(Sim_Data_event_data_title);

        Sim_Data_event_data_QL = new QLineEdit(verticalLayoutWidget_20);
        Sim_Data_event_data_QL->setObjectName(QStringLiteral("Sim_Data_event_data_QL"));
        Sim_Data_event_data_QL->setFont(font7);
        Sim_Data_event_data_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_event_data_QV->addWidget(Sim_Data_event_data_QL);


        Sim_Data_QV_2->addWidget(Sim_Data_event_data_QW);

        Sim_Data_armor_id_QW = new QWidget(verticalLayoutWidget_11);
        Sim_Data_armor_id_QW->setObjectName(QStringLiteral("Sim_Data_armor_id_QW"));
        Sim_Data_armor_id_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_24 = new QWidget(Sim_Data_armor_id_QW);
        verticalLayoutWidget_24->setObjectName(QStringLiteral("verticalLayoutWidget_24"));
        verticalLayoutWidget_24->setGeometry(QRect(0, 0, 111, 84));
        Sim_Data_armor_id_QV = new QVBoxLayout(verticalLayoutWidget_24);
        Sim_Data_armor_id_QV->setObjectName(QStringLiteral("Sim_Data_armor_id_QV"));
        Sim_Data_armor_id_QV->setContentsMargins(15, 15, 15, 15);
        Sim_Data_armor_id_title = new QLabel(verticalLayoutWidget_24);
        Sim_Data_armor_id_title->setObjectName(QStringLiteral("Sim_Data_armor_id_title"));
        sizePolicy1.setHeightForWidth(Sim_Data_armor_id_title->sizePolicy().hasHeightForWidth());
        Sim_Data_armor_id_title->setSizePolicy(sizePolicy1);
        Sim_Data_armor_id_title->setFont(font6);
        Sim_Data_armor_id_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        Sim_Data_armor_id_QV->addWidget(Sim_Data_armor_id_title);

        Sim_Data_armor_id_QL = new QLineEdit(verticalLayoutWidget_24);
        Sim_Data_armor_id_QL->setObjectName(QStringLiteral("Sim_Data_armor_id_QL"));
        Sim_Data_armor_id_QL->setFont(font7);
        Sim_Data_armor_id_QL->setStyleSheet(QString::fromUtf8("border: none;          /* \351\232\220\350\227\217\350\276\271\346\241\206 */\n"
"background-color: rgb(240, 240, 240);\n"
"color: rgb(58, 58, 58);"));

        Sim_Data_armor_id_QV->addWidget(Sim_Data_armor_id_QL);


        Sim_Data_QV_2->addWidget(Sim_Data_armor_id_QW);

        Sim_Data_TITLE = new QLabel(Sim_Data_QW);
        Sim_Data_TITLE->setObjectName(QStringLiteral("Sim_Data_TITLE"));
        Sim_Data_TITLE->setGeometry(QRect(20, 30, 201, 61));
        sizePolicy1.setHeightForWidth(Sim_Data_TITLE->sizePolicy().hasHeightForWidth());
        Sim_Data_TITLE->setSizePolicy(sizePolicy1);
        QFont font8;
        font8.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font8.setPointSize(24);
        Sim_Data_TITLE->setFont(font8);
        Sim_Data_TITLE->setStyleSheet(QStringLiteral("color: rgb(100, 100, 100);"));
        Sim_Data_tips = new QLabel(Sim_Data_QW);
        Sim_Data_tips->setObjectName(QStringLiteral("Sim_Data_tips"));
        Sim_Data_tips->setGeometry(QRect(20, 100, 251, 31));
        sizePolicy1.setHeightForWidth(Sim_Data_tips->sizePolicy().hasHeightForWidth());
        Sim_Data_tips->setSizePolicy(sizePolicy1);
        QFont font9;
        font9.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font9.setPointSize(10);
        Sim_Data_tips->setFont(font9);
        Sim_Data_tips->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));
        Sim_Data_change_QP = new QPushButton(Sim_Data_QW);
        Sim_Data_change_QP->setObjectName(QStringLiteral("Sim_Data_change_QP"));
        Sim_Data_change_QP->setGeometry(QRect(20, 630, 281, 41));
        Sim_Data_change_QP->setFont(font9);
        Sim_Data_change_QP->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border-radius: 10px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    border: 2px solid rgb(240, 240, 240); /* \351\273\230\350\256\244\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    background-color: rgb(240, 240, 240); /* \351\273\230\350\256\244\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(145, 146, 152); /* \351\273\230\350\256\244\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    padding: 5px 10px; /* \345\206\205\350\276\271\350\267\235\357\274\214\350\260\203\346\225\264\346\214\211\351\222\256\345\206\205\345\256\271\344\275\215\347\275\256 */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: rgb(127, 176, 255); /* \351\274\240\346\240\207\346\202\254\345\201\234\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239);; /* \346\202\254\345\201"
                        "\234\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: white; /* \346\214\211\344\270\213\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
""));
        DEBUG = new QWidget(centralwidget);
        DEBUG->setObjectName(QStringLiteral("DEBUG"));
        DEBUG->setGeometry(QRect(850, 20, 691, 361));
        sizePolicy.setHeightForWidth(DEBUG->sizePolicy().hasHeightForWidth());
        DEBUG->setSizePolicy(sizePolicy);
        DEBUG->setLayoutDirection(Qt::LeftToRight);
        DEBUG->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        DEBUG_TITLE = new QLabel(DEBUG);
        DEBUG_TITLE->setObjectName(QStringLiteral("DEBUG_TITLE"));
        DEBUG_TITLE->setGeometry(QRect(30, 20, 280, 60));
        sizePolicy1.setHeightForWidth(DEBUG_TITLE->sizePolicy().hasHeightForWidth());
        DEBUG_TITLE->setSizePolicy(sizePolicy1);
        DEBUG_TITLE->setFont(font8);
        DEBUG_TITLE->setStyleSheet(QStringLiteral("color: rgb(58, 58, 58);"));
        horizontalLayoutWidget_8 = new QWidget(DEBUG);
        horizontalLayoutWidget_8->setObjectName(QStringLiteral("horizontalLayoutWidget_8"));
        horizontalLayoutWidget_8->setGeometry(QRect(20, 90, 611, 111));
        rules_QH = new QHBoxLayout(horizontalLayoutWidget_8);
        rules_QH->setSpacing(20);
        rules_QH->setObjectName(QStringLiteral("rules_QH"));
        rules_QH->setContentsMargins(5, 5, 5, 5);
        rules_date_QW = new QWidget(horizontalLayoutWidget_8);
        rules_date_QW->setObjectName(QStringLiteral("rules_date_QW"));
        rules_date_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_12 = new QWidget(rules_date_QW);
        verticalLayoutWidget_12->setObjectName(QStringLiteral("verticalLayoutWidget_12"));
        verticalLayoutWidget_12->setGeometry(QRect(0, 0, 161, 91));
        rules_date_QV = new QVBoxLayout(verticalLayoutWidget_12);
        rules_date_QV->setObjectName(QStringLiteral("rules_date_QV"));
        rules_date_QV->setContentsMargins(15, 15, 15, 15);
        rules_date_title = new QLabel(verticalLayoutWidget_12);
        rules_date_title->setObjectName(QStringLiteral("rules_date_title"));
        sizePolicy1.setHeightForWidth(rules_date_title->sizePolicy().hasHeightForWidth());
        rules_date_title->setSizePolicy(sizePolicy1);
        rules_date_title->setFont(font3);
        rules_date_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        rules_date_QV->addWidget(rules_date_title);

        rules_date_state = new QLabel(verticalLayoutWidget_12);
        rules_date_state->setObjectName(QStringLiteral("rules_date_state"));
        sizePolicy1.setHeightForWidth(rules_date_state->sizePolicy().hasHeightForWidth());
        rules_date_state->setSizePolicy(sizePolicy1);
        QFont font10;
        font10.setFamily(QString::fromUtf8("\351\273\221\344\275\223"));
        font10.setPointSize(12);
        font10.setUnderline(false);
        rules_date_state->setFont(font10);
        rules_date_state->setStyleSheet(QStringLiteral("color: rgb(15, 114, 239);"));

        rules_date_QV->addWidget(rules_date_state);


        rules_QH->addWidget(rules_date_QW);

        rules_version_QW = new QWidget(horizontalLayoutWidget_8);
        rules_version_QW->setObjectName(QStringLiteral("rules_version_QW"));
        rules_version_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_13 = new QWidget(rules_version_QW);
        verticalLayoutWidget_13->setObjectName(QStringLiteral("verticalLayoutWidget_13"));
        verticalLayoutWidget_13->setGeometry(QRect(0, 0, 161, 91));
        rules_version_QV = new QVBoxLayout(verticalLayoutWidget_13);
        rules_version_QV->setObjectName(QStringLiteral("rules_version_QV"));
        rules_version_QV->setContentsMargins(15, 15, 15, 15);
        rules_version_title = new QLabel(verticalLayoutWidget_13);
        rules_version_title->setObjectName(QStringLiteral("rules_version_title"));
        sizePolicy1.setHeightForWidth(rules_version_title->sizePolicy().hasHeightForWidth());
        rules_version_title->setSizePolicy(sizePolicy1);
        rules_version_title->setFont(font3);
        rules_version_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        rules_version_QV->addWidget(rules_version_title);

        rules_version_state = new QLabel(verticalLayoutWidget_13);
        rules_version_state->setObjectName(QStringLiteral("rules_version_state"));
        sizePolicy1.setHeightForWidth(rules_version_state->sizePolicy().hasHeightForWidth());
        rules_version_state->setSizePolicy(sizePolicy1);
        rules_version_state->setFont(font10);
        rules_version_state->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        rules_version_QV->addWidget(rules_version_state);


        rules_QH->addWidget(rules_version_QW);

        rules_nums_QW = new QWidget(horizontalLayoutWidget_8);
        rules_nums_QW->setObjectName(QStringLiteral("rules_nums_QW"));
        rules_nums_QW->setStyleSheet(QLatin1String("background-color: rgb(240, 240, 240);\n"
"border-radius: 10px;\n"
""));
        verticalLayoutWidget_14 = new QWidget(rules_nums_QW);
        verticalLayoutWidget_14->setObjectName(QStringLiteral("verticalLayoutWidget_14"));
        verticalLayoutWidget_14->setGeometry(QRect(0, 0, 161, 91));
        rules_nums_QV = new QVBoxLayout(verticalLayoutWidget_14);
        rules_nums_QV->setObjectName(QStringLiteral("rules_nums_QV"));
        rules_nums_QV->setContentsMargins(15, 15, 15, 15);
        rules_nums_title = new QLabel(verticalLayoutWidget_14);
        rules_nums_title->setObjectName(QStringLiteral("rules_nums_title"));
        sizePolicy1.setHeightForWidth(rules_nums_title->sizePolicy().hasHeightForWidth());
        rules_nums_title->setSizePolicy(sizePolicy1);
        rules_nums_title->setFont(font3);
        rules_nums_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        rules_nums_QV->addWidget(rules_nums_title);

        rules_nums_value = new QLabel(verticalLayoutWidget_14);
        rules_nums_value->setObjectName(QStringLiteral("rules_nums_value"));
        sizePolicy1.setHeightForWidth(rules_nums_value->sizePolicy().hasHeightForWidth());
        rules_nums_value->setSizePolicy(sizePolicy1);
        rules_nums_value->setFont(font10);
        rules_nums_value->setStyleSheet(QStringLiteral("color: rgb(255, 81, 105);"));

        rules_nums_QV->addWidget(rules_nums_value);


        rules_QH->addWidget(rules_nums_QW);

        rules_path_QL = new QLineEdit(DEBUG);
        rules_path_QL->setObjectName(QStringLiteral("rules_path_QL"));
        rules_path_QL->setGeometry(QRect(20, 210, 611, 41));
        rules_path_QL->setFont(font7);
        rules_path_QL->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: none; /* \351\232\220\350\227\217\345\216\237\345\247\213\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(58, 58, 58); /* \345\255\227\344\275\223\351\242\234\350\211\262 */\n"
"    border-radius: 20px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    padding: 0px 40px; /* \345\220\221\345\244\226\346\211\251\345\261\225 40 \345\203\217\347\264\240\347\251\272\351\227\264 */\n"
"    height: 30px; /* \350\256\276\347\275\256\346\216\247\344\273\266\351\253\230\345\272\246 */\n"
"    margin: 0; /* \351\230\262\346\255\242\351\242\235\345\244\226\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"}\n"
"\n"
"QLineEdit::focus {\n"
"    outline: 2px solid rgb(0, 193, 139); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350\276\271\346\241\206 */\n"
"    background-color: rgb(250, 250, 250); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350"
                        "\203\214\346\231\257\351\242\234\350\211\262 */\n"
"}\n"
""));
        refresh_rules_QP = new QPushButton(DEBUG);
        refresh_rules_QP->setObjectName(QStringLiteral("refresh_rules_QP"));
        refresh_rules_QP->setGeometry(QRect(200, 300, 161, 34));
        refresh_rules_QP->setFont(font9);
        refresh_rules_QP->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border-radius: 10px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    border: 2px solid rgb(15, 114, 239); /* \351\273\230\350\256\244\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    background-color: white; /* \351\273\230\350\256\244\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239); /* \351\273\230\350\256\244\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    padding: 5px 10px; /* \345\206\205\350\276\271\350\267\235\357\274\214\350\260\203\346\225\264\346\214\211\351\222\256\345\206\205\345\256\271\344\275\215\347\275\256 */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: rgb(127, 176, 255); /* \351\274\240\346\240\207\346\202\254\345\201\234\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266"
                        "\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: white; /* \346\214\211\344\270\213\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
""));
        reload_rules_QP = new QPushButton(DEBUG);
        reload_rules_QP->setObjectName(QStringLiteral("reload_rules_QP"));
        reload_rules_QP->setGeometry(QRect(30, 300, 161, 34));
        reload_rules_QP->setFont(font9);
        reload_rules_QP->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border-radius: 10px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    border: 2px solid rgb(15, 114, 239); /* \351\273\230\350\256\244\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    background-color: white; /* \351\273\230\350\256\244\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239); /* \351\273\230\350\256\244\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    padding: 5px 10px; /* \345\206\205\350\276\271\350\267\235\357\274\214\350\260\203\346\225\264\346\214\211\351\222\256\345\206\205\345\256\271\344\275\215\347\275\256 */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: rgb(127, 176, 255); /* \351\274\240\346\240\207\346\202\254\345\201\234\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266"
                        "\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: white; /* \346\214\211\344\270\213\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
""));
        rules_path_tips = new QLabel(DEBUG);
        rules_path_tips->setObjectName(QStringLiteral("rules_path_tips"));
        rules_path_tips->setGeometry(QRect(30, 260, 331, 31));
        sizePolicy1.setHeightForWidth(rules_path_tips->sizePolicy().hasHeightForWidth());
        rules_path_tips->setSizePolicy(sizePolicy1);
        rules_path_tips->setFont(font9);
        rules_path_tips->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));
        goal_pose_debug_QW = new QWidget(centralwidget);
        goal_pose_debug_QW->setObjectName(QStringLiteral("goal_pose_debug_QW"));
        goal_pose_debug_QW->setGeometry(QRect(850, 650, 691, 221));
        sizePolicy.setHeightForWidth(goal_pose_debug_QW->sizePolicy().hasHeightForWidth());
        goal_pose_debug_QW->setSizePolicy(sizePolicy);
        goal_pose_debug_QW->setLayoutDirection(Qt::LeftToRight);
        goal_pose_debug_QW->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        goal_pose_title = new QLabel(goal_pose_debug_QW);
        goal_pose_title->setObjectName(QStringLiteral("goal_pose_title"));
        goal_pose_title->setGeometry(QRect(20, 20, 201, 61));
        sizePolicy1.setHeightForWidth(goal_pose_title->sizePolicy().hasHeightForWidth());
        goal_pose_title->setSizePolicy(sizePolicy1);
        QFont font11;
        font11.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font11.setPointSize(16);
        goal_pose_title->setFont(font11);
        goal_pose_title->setStyleSheet(QStringLiteral("color: rgb(100, 100, 100);"));
        horizontalLayoutWidget_5 = new QWidget(goal_pose_debug_QW);
        horizontalLayoutWidget_5->setObjectName(QStringLiteral("horizontalLayoutWidget_5"));
        horizontalLayoutWidget_5->setGeometry(QRect(20, 110, 343, 51));
        goal_pose_QH = new QHBoxLayout(horizontalLayoutWidget_5);
        goal_pose_QH->setObjectName(QStringLiteral("goal_pose_QH"));
        goal_pose_QH->setContentsMargins(0, 0, 0, 0);
        goal_pose_X = new QLineEdit(horizontalLayoutWidget_5);
        goal_pose_X->setObjectName(QStringLiteral("goal_pose_X"));
        sizePolicy.setHeightForWidth(goal_pose_X->sizePolicy().hasHeightForWidth());
        goal_pose_X->setSizePolicy(sizePolicy);
        QFont font12;
        font12.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font12.setPointSize(10);
        font12.setBold(false);
        font12.setWeight(50);
        goal_pose_X->setFont(font12);
        goal_pose_X->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: none; /* \351\232\220\350\227\217\345\216\237\345\247\213\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(58, 58, 58); /* \345\255\227\344\275\223\351\242\234\350\211\262 */\n"
"    border-radius: 20px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    padding: 0px 25px; /* \345\220\221\345\244\226\346\211\251\345\261\225 40 \345\203\217\347\264\240\347\251\272\351\227\264 */\n"
"    height: 30px; /* \350\256\276\347\275\256\346\216\247\344\273\266\351\253\230\345\272\246 */\n"
"    margin: 0; /* \351\230\262\346\255\242\351\242\235\345\244\226\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"}\n"
"\n"
"QLineEdit::focus {\n"
"    outline: 2px solid rgb(0, 193, 139); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350\276\271\346\241\206 */\n"
"    background-color: rgb(250, 250, 250); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350"
                        "\203\214\346\231\257\351\242\234\350\211\262 */\n"
"}\n"
""));

        goal_pose_QH->addWidget(goal_pose_X);

        goal_pose_Y = new QLineEdit(horizontalLayoutWidget_5);
        goal_pose_Y->setObjectName(QStringLiteral("goal_pose_Y"));
        sizePolicy.setHeightForWidth(goal_pose_Y->sizePolicy().hasHeightForWidth());
        goal_pose_Y->setSizePolicy(sizePolicy);
        goal_pose_Y->setFont(font12);
        goal_pose_Y->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: none; /* \351\232\220\350\227\217\345\216\237\345\247\213\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(58, 58, 58); /* \345\255\227\344\275\223\351\242\234\350\211\262 */\n"
"    border-radius: 20px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    padding: 0px 25px; /* \345\220\221\345\244\226\346\211\251\345\261\225 40 \345\203\217\347\264\240\347\251\272\351\227\264 */\n"
"    height: 30px; /* \350\256\276\347\275\256\346\216\247\344\273\266\351\253\230\345\272\246 */\n"
"    margin: 0; /* \351\230\262\346\255\242\351\242\235\345\244\226\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"}\n"
"\n"
"QLineEdit::focus {\n"
"    outline: 2px solid rgb(0, 193, 139); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350\276\271\346\241\206 */\n"
"    background-color: rgb(250, 250, 250); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350"
                        "\203\214\346\231\257\351\242\234\350\211\262 */\n"
"}\n"
""));

        goal_pose_QH->addWidget(goal_pose_Y);

        goal_pose_YAW = new QLineEdit(horizontalLayoutWidget_5);
        goal_pose_YAW->setObjectName(QStringLiteral("goal_pose_YAW"));
        sizePolicy.setHeightForWidth(goal_pose_YAW->sizePolicy().hasHeightForWidth());
        goal_pose_YAW->setSizePolicy(sizePolicy);
        goal_pose_YAW->setFont(font12);
        goal_pose_YAW->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: none; /* \351\232\220\350\227\217\345\216\237\345\247\213\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(58, 58, 58); /* \345\255\227\344\275\223\351\242\234\350\211\262 */\n"
"    border-radius: 20px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    padding: 0px 25px; /* \345\220\221\345\244\226\346\211\251\345\261\225 40 \345\203\217\347\264\240\347\251\272\351\227\264 */\n"
"    height: 30px; /* \350\256\276\347\275\256\346\216\247\344\273\266\351\253\230\345\272\246 */\n"
"    margin: 0; /* \351\230\262\346\255\242\351\242\235\345\244\226\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"}\n"
"\n"
"QLineEdit::focus {\n"
"    outline: 2px solid rgb(0, 193, 139); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350\276\271\346\241\206 */\n"
"    background-color: rgb(250, 250, 250); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350"
                        "\203\214\346\231\257\351\242\234\350\211\262 */\n"
"}\n"
""));

        goal_pose_QH->addWidget(goal_pose_YAW);

        goal_pose_tips = new QLabel(goal_pose_debug_QW);
        goal_pose_tips->setObjectName(QStringLiteral("goal_pose_tips"));
        goal_pose_tips->setGeometry(QRect(20, 70, 331, 31));
        sizePolicy1.setHeightForWidth(goal_pose_tips->sizePolicy().hasHeightForWidth());
        goal_pose_tips->setSizePolicy(sizePolicy1);
        goal_pose_tips->setFont(font9);
        goal_pose_tips->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));
        horizontalLayoutWidget_6 = new QWidget(goal_pose_debug_QW);
        horizontalLayoutWidget_6->setObjectName(QStringLiteral("horizontalLayoutWidget_6"));
        horizontalLayoutWidget_6->setGeometry(QRect(400, 110, 131, 51));
        goal_pose_send_way_QH = new QHBoxLayout(horizontalLayoutWidget_6);
        goal_pose_send_way_QH->setSpacing(15);
        goal_pose_send_way_QH->setObjectName(QStringLiteral("goal_pose_send_way_QH"));
        goal_pose_send_way_QH->setContentsMargins(0, 0, 0, 0);
        goal_pose_send_way_title = new QLabel(horizontalLayoutWidget_6);
        goal_pose_send_way_title->setObjectName(QStringLiteral("goal_pose_send_way_title"));
        sizePolicy1.setHeightForWidth(goal_pose_send_way_title->sizePolicy().hasHeightForWidth());
        goal_pose_send_way_title->setSizePolicy(sizePolicy1);
        goal_pose_send_way_title->setFont(font9);
        goal_pose_send_way_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        goal_pose_send_way_QH->addWidget(goal_pose_send_way_title);

        goal_pose_send_way_QSL = new QSlider(horizontalLayoutWidget_6);
        goal_pose_send_way_QSL->setObjectName(QStringLiteral("goal_pose_send_way_QSL"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(goal_pose_send_way_QSL->sizePolicy().hasHeightForWidth());
        goal_pose_send_way_QSL->setSizePolicy(sizePolicy2);
        goal_pose_send_way_QSL->setMaximumSize(QSize(40, 16777215));
        goal_pose_send_way_QSL->setStyleSheet(QString::fromUtf8("QSlider {\n"
"    background: transparent; /* \346\273\221\345\212\250\346\235\241\346\225\264\344\275\223\350\203\214\346\231\257 */\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"    height: 10px; /* \346\273\221\345\212\250\346\247\275\347\232\204\351\253\230\345\272\246 */\n"
"    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 rgb(220, 220, 220), stop:1 rgb(240, 240, 240)); /* \346\270\220\345\217\230\346\273\221\345\212\250\346\247\275\351\242\234\350\211\262 */\n"
"    border-radius: 7px; /* \346\273\221\345\212\250\346\247\275\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    width: 20px; /* \346\273\221\345\235\227\345\256\275\345\272\246 */\n"
"    height: 20px; /* \346\273\221\345\235\227\351\253\230\345\272\246 */\n"
"    margin: -5px 0; /* \350\260\203\346\225\264\346\273\221\345\235\227\345\257\271\351\275\220 */\n"
"    background-color: rgb(145, 146, 152);\n"
"    border-radius: 10px; /* \346\273\221\345\235\227\345\256\214\345\205\250\345\234\206\345\275"
                        "\242 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"	\n"
"	background-color: rgb(15, 114, 239);\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    \n"
"	\n"
"	background-color: rgb(0, 193, 139);\n"
"    border-radius: 5px; /* \345\267\262\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: rgb(220, 220, 220); /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\347\232\204\351\242\234\350\211\262 */\n"
"    border-radius: 5px; /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
""));
        goal_pose_send_way_QSL->setMaximum(1);
        goal_pose_send_way_QSL->setOrientation(Qt::Horizontal);

        goal_pose_send_way_QH->addWidget(goal_pose_send_way_QSL);

        goal_pose_send_QP = new QPushButton(goal_pose_debug_QW);
        goal_pose_send_QP->setObjectName(QStringLiteral("goal_pose_send_QP"));
        goal_pose_send_QP->setGeometry(QRect(550, 113, 121, 41));
        goal_pose_send_QP->setFont(font9);
        goal_pose_send_QP->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    border-radius: 10px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    border: 2px solid rgb(240, 240, 240); /* \351\273\230\350\256\244\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"    background-color: rgb(240, 240, 240); /* \351\273\230\350\256\244\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(145, 146, 152); /* \351\273\230\350\256\244\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    padding: 5px 10px; /* \345\206\205\350\276\271\350\267\235\357\274\214\350\260\203\346\225\264\346\214\211\351\222\256\345\206\205\345\256\271\344\275\215\347\275\256 */\n"
"}\n"
"\n"
"QPushButton:hover {\n"
"    background-color: rgb(127, 176, 255); /* \351\274\240\346\240\207\346\202\254\345\201\234\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(15, 114, 239);; /* \346\202\254\345\201\234\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239);; /* \346\202\254\345\201"
                        "\234\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
"\n"
"QPushButton:pressed {\n"
"    background-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: white; /* \346\214\211\344\270\213\346\227\266\346\226\207\345\255\227\351\242\234\350\211\262 */\n"
"    border-color: rgb(15, 114, 239); /* \346\214\211\344\270\213\346\227\266\350\276\271\346\241\206\351\242\234\350\211\262 */\n"
"}\n"
""));
        goal_pose_send_way_tips = new QLabel(goal_pose_debug_QW);
        goal_pose_send_way_tips->setObjectName(QStringLiteral("goal_pose_send_way_tips"));
        goal_pose_send_way_tips->setGeometry(QRect(20, 170, 641, 31));
        sizePolicy1.setHeightForWidth(goal_pose_send_way_tips->sizePolicy().hasHeightForWidth());
        goal_pose_send_way_tips->setSizePolicy(sizePolicy1);
        goal_pose_send_way_tips->setFont(font9);
        goal_pose_send_way_tips->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));
        datadebug_QW = new QWidget(centralwidget);
        datadebug_QW->setObjectName(QStringLiteral("datadebug_QW"));
        datadebug_QW->setGeometry(QRect(850, 410, 691, 221));
        sizePolicy.setHeightForWidth(datadebug_QW->sizePolicy().hasHeightForWidth());
        datadebug_QW->setSizePolicy(sizePolicy);
        datadebug_QW->setLayoutDirection(Qt::LeftToRight);
        datadebug_QW->setStyleSheet(QLatin1String("background-color: rgb(243, 246, 249);\n"
"border-radius: 25px;\n"
""));
        horizontalLayoutWidget_4 = new QWidget(datadebug_QW);
        horizontalLayoutWidget_4->setObjectName(QStringLiteral("horizontalLayoutWidget_4"));
        horizontalLayoutWidget_4->setGeometry(QRect(20, 140, 499, 80));
        debug_swtich_QH = new QHBoxLayout(horizontalLayoutWidget_4);
        debug_swtich_QH->setSpacing(40);
        debug_swtich_QH->setObjectName(QStringLiteral("debug_swtich_QH"));
        debug_swtich_QH->setContentsMargins(0, 0, 0, 0);
        debug_send_QH = new QHBoxLayout();
        debug_send_QH->setSpacing(40);
        debug_send_QH->setObjectName(QStringLiteral("debug_send_QH"));
        debug_send_title = new QLabel(horizontalLayoutWidget_4);
        debug_send_title->setObjectName(QStringLiteral("debug_send_title"));
        sizePolicy1.setHeightForWidth(debug_send_title->sizePolicy().hasHeightForWidth());
        debug_send_title->setSizePolicy(sizePolicy1);
        QFont font13;
        font13.setFamily(QString::fromUtf8("\346\226\207\346\263\211\351\251\277\345\276\256\347\261\263\351\273\221"));
        font13.setPointSize(14);
        debug_send_title->setFont(font13);
        debug_send_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        debug_send_QH->addWidget(debug_send_title);

        debug_send_QSL = new QSlider(horizontalLayoutWidget_4);
        debug_send_QSL->setObjectName(QStringLiteral("debug_send_QSL"));
        sizePolicy2.setHeightForWidth(debug_send_QSL->sizePolicy().hasHeightForWidth());
        debug_send_QSL->setSizePolicy(sizePolicy2);
        debug_send_QSL->setMaximumSize(QSize(40, 16666));
        debug_send_QSL->setStyleSheet(QString::fromUtf8("QSlider {\n"
"    background: transparent; /* \346\273\221\345\212\250\346\235\241\346\225\264\344\275\223\350\203\214\346\231\257 */\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"    height: 10px; /* \346\273\221\345\212\250\346\247\275\347\232\204\351\253\230\345\272\246 */\n"
"    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 rgb(220, 220, 220), stop:1 rgb(240, 240, 240)); /* \346\270\220\345\217\230\346\273\221\345\212\250\346\247\275\351\242\234\350\211\262 */\n"
"    border-radius: 7px; /* \346\273\221\345\212\250\346\247\275\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    width: 20px; /* \346\273\221\345\235\227\345\256\275\345\272\246 */\n"
"    height: 20px; /* \346\273\221\345\235\227\351\253\230\345\272\246 */\n"
"    margin: -5px 0; /* \350\260\203\346\225\264\346\273\221\345\235\227\345\257\271\351\275\220 */\n"
"    background-color: rgb(15, 114, 239);\n"
"    border-radius: 10px; /* \346\273\221\345\235\227\345\256\214\345\205\250\345\234\206\345\275"
                        "\242 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"	\n"
"	background-color: rgb(15, 114, 239);\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    \n"
"	\n"
"	background-color: rgb(0, 193, 139);\n"
"    border-radius: 5px; /* \345\267\262\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: rgb(220, 220, 220); /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\347\232\204\351\242\234\350\211\262 */\n"
"    border-radius: 5px; /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
""));
        debug_send_QSL->setMaximum(1);
        debug_send_QSL->setOrientation(Qt::Horizontal);

        debug_send_QH->addWidget(debug_send_QSL);


        debug_swtich_QH->addLayout(debug_send_QH);

        debug_listen_QH = new QHBoxLayout();
        debug_listen_QH->setSpacing(40);
        debug_listen_QH->setObjectName(QStringLiteral("debug_listen_QH"));
        debug_listen_title = new QLabel(horizontalLayoutWidget_4);
        debug_listen_title->setObjectName(QStringLiteral("debug_listen_title"));
        sizePolicy1.setHeightForWidth(debug_listen_title->sizePolicy().hasHeightForWidth());
        debug_listen_title->setSizePolicy(sizePolicy1);
        debug_listen_title->setFont(font13);
        debug_listen_title->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));

        debug_listen_QH->addWidget(debug_listen_title);

        debug_listen_QSL = new QSlider(horizontalLayoutWidget_4);
        debug_listen_QSL->setObjectName(QStringLiteral("debug_listen_QSL"));
        sizePolicy2.setHeightForWidth(debug_listen_QSL->sizePolicy().hasHeightForWidth());
        debug_listen_QSL->setSizePolicy(sizePolicy2);
        debug_listen_QSL->setMaximumSize(QSize(40, 16777215));
        debug_listen_QSL->setStyleSheet(QString::fromUtf8("QSlider {\n"
"    background: transparent; /* \346\273\221\345\212\250\346\235\241\346\225\264\344\275\223\350\203\214\346\231\257 */\n"
"}\n"
"\n"
"QSlider::groove:horizontal {\n"
"    height: 10px; /* \346\273\221\345\212\250\346\247\275\347\232\204\351\253\230\345\272\246 */\n"
"    background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 rgb(220, 220, 220), stop:1 rgb(240, 240, 240)); /* \346\270\220\345\217\230\346\273\221\345\212\250\346\247\275\351\242\234\350\211\262 */\n"
"    border-radius: 7px; /* \346\273\221\345\212\250\346\247\275\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal {\n"
"    width: 20px; /* \346\273\221\345\235\227\345\256\275\345\272\246 */\n"
"    height: 20px; /* \346\273\221\345\235\227\351\253\230\345\272\246 */\n"
"    margin: -5px 0; /* \350\260\203\346\225\264\346\273\221\345\235\227\345\257\271\351\275\220 */\n"
"    background-color: rgb(15, 114, 239);\n"
"    border-radius: 10px; /* \346\273\221\345\235\227\345\256\214\345\205\250\345\234\206\345\275"
                        "\242 */\n"
"}\n"
"\n"
"QSlider::handle:horizontal:hover {\n"
"	\n"
"	background-color: rgb(15, 114, 239);\n"
"}\n"
"\n"
"QSlider::sub-page:horizontal {\n"
"    \n"
"	\n"
"	background-color: rgb(0, 193, 139);\n"
"    border-radius: 5px; /* \345\267\262\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
"\n"
"QSlider::add-page:horizontal {\n"
"    background: rgb(220, 220, 220); /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\347\232\204\351\242\234\350\211\262 */\n"
"    border-radius: 5px; /* \346\234\252\346\273\221\350\277\207\351\203\250\345\210\206\345\234\206\350\247\222 */\n"
"}\n"
""));
        debug_listen_QSL->setMaximum(1);
        debug_listen_QSL->setOrientation(Qt::Horizontal);

        debug_listen_QH->addWidget(debug_listen_QSL);


        debug_swtich_QH->addLayout(debug_listen_QH);

        pub_topic_QL = new QLineEdit(datadebug_QW);
        pub_topic_QL->setObjectName(QStringLiteral("pub_topic_QL"));
        pub_topic_QL->setGeometry(QRect(20, 90, 611, 41));
        pub_topic_QL->setFont(font7);
        pub_topic_QL->setStyleSheet(QString::fromUtf8("QLineEdit {\n"
"    border: none; /* \351\232\220\350\227\217\345\216\237\345\247\213\350\276\271\346\241\206 */\n"
"    background-color: rgb(240, 240, 240); /* \350\203\214\346\231\257\351\242\234\350\211\262 */\n"
"    color: rgb(58, 58, 58); /* \345\255\227\344\275\223\351\242\234\350\211\262 */\n"
"    border-radius: 20px; /* \345\234\206\350\247\222\345\215\212\345\276\204 */\n"
"    padding: 0px 40px; /* \345\220\221\345\244\226\346\211\251\345\261\225 40 \345\203\217\347\264\240\347\251\272\351\227\264 */\n"
"    height: 30px; /* \350\256\276\347\275\256\346\216\247\344\273\266\351\253\230\345\272\246 */\n"
"    margin: 0; /* \351\230\262\346\255\242\351\242\235\345\244\226\347\232\204\345\244\226\350\276\271\350\267\235 */\n"
"}\n"
"\n"
"QLineEdit::focus {\n"
"    outline: 2px solid rgb(0, 193, 139); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350\276\271\346\241\206 */\n"
"    background-color: rgb(250, 250, 250); /* \347\204\246\347\202\271\347\212\266\346\200\201\347\232\204\350"
                        "\203\214\346\231\257\351\242\234\350\211\262 */\n"
"}\n"
""));
        datadebug_title = new QLabel(datadebug_QW);
        datadebug_title->setObjectName(QStringLiteral("datadebug_title"));
        datadebug_title->setGeometry(QRect(20, 0, 201, 61));
        sizePolicy1.setHeightForWidth(datadebug_title->sizePolicy().hasHeightForWidth());
        datadebug_title->setSizePolicy(sizePolicy1);
        datadebug_title->setFont(font11);
        datadebug_title->setStyleSheet(QStringLiteral("color: rgb(100, 100, 100);"));
        debug_tips = new QLabel(datadebug_QW);
        debug_tips->setObjectName(QStringLiteral("debug_tips"));
        debug_tips->setGeometry(QRect(20, 50, 511, 31));
        sizePolicy1.setHeightForWidth(debug_tips->sizePolicy().hasHeightForWidth());
        debug_tips->setSizePolicy(sizePolicy1);
        debug_tips->setFont(font9);
        debug_tips->setStyleSheet(QStringLiteral("color: rgb(145, 146, 152);"));
        DEBUG_PAGE->setCentralWidget(centralwidget);

        retranslateUi(DEBUG_PAGE);

        QMetaObject::connectSlotsByName(DEBUG_PAGE);
    } // setupUi

    void retranslateUi(QMainWindow *DEBUG_PAGE)
    {
        DEBUG_PAGE->setWindowTitle(QApplication::translate("DEBUG_PAGE", "MainWindow", nullptr));
        progress_stage_remain_time_bar_title->setText(QApplication::translate("DEBUG_PAGE", "\345\211\251\344\275\231\346\227\266\351\227\264", nullptr));
        progress_stage_remain_time_bar_value->setFormat(QApplication::translate("DEBUG_PAGE", "%v", nullptr));
        progress_own_robot_HP_bar_title->setText(QApplication::translate("DEBUG_PAGE", "\345\223\250\345\205\265\350\241\200\351\207\217", nullptr));
        progress_own_robot_HP_bar_value->setFormat(QApplication::translate("DEBUG_PAGE", "%v", nullptr));
        progress_own_base_HP_bar_title->setText(QApplication::translate("DEBUG_PAGE", "\345\267\261\346\226\271\345\237\272\345\234\260", nullptr));
        progress_own_base_HP_bar_value->setFormat(QApplication::translate("DEBUG_PAGE", "%v", nullptr));
        progress_enemy_base_HP_bar_title->setText(QApplication::translate("DEBUG_PAGE", "\346\225\214\346\226\271\345\237\272\345\234\260", nullptr));
        progress_enemy_base_HP_bar_value->setFormat(QApplication::translate("DEBUG_PAGE", "%v", nullptr));
        progress_shoot_num_bar_title->setText(QApplication::translate("DEBUG_PAGE", "\345\255\220\345\274\271\346\225\260\351\207\217", nullptr));
        progress_shoot_num_bar_value->setFormat(QApplication::translate("DEBUG_PAGE", "%v", nullptr));
        progress_nav_state_title->setText(QApplication::translate("DEBUG_PAGE", "\345\257\274\350\210\252\350\277\233\345\272\246", nullptr));
        progress_nav_state_value->setFormat(QApplication::translate("DEBUG_PAGE", "%p%", nullptr));
        home_page->setText(QString());
        rules_page->setText(QString());
        debug_page->setText(QString());
        setting_page->setText(QString());
        monitor_task_id_title->setText(QApplication::translate("DEBUG_PAGE", "\345\275\223\345\211\215\344\273\273\345\212\241", nullptr));
        monitor_task_id_state->setText(QApplication::translate("DEBUG_PAGE", "\344\273\273\345\212\241\347\274\226\345\217\26708", nullptr));
        monitor_goal_pose_title->setText(QApplication::translate("DEBUG_PAGE", "\350\210\252\347\202\271\345\235\220\346\240\207", nullptr));
        monitor_goal_pose_state->setText(QApplication::translate("DEBUG_PAGE", "3.2 7.6", nullptr));
        monitor_vision_status_title->setText(QApplication::translate("DEBUG_PAGE", "\350\247\206\350\247\211\347\212\266\346\200\201", nullptr));
        monitor_vision_status_sate->setText(QApplication::translate("DEBUG_PAGE", "\346\243\200\346\265\213\345\210\260\346\225\214\346\226\271", nullptr));
        monitor_enemy_pose_title->setText(QApplication::translate("DEBUG_PAGE", "\346\225\214\346\226\271\344\275\215\347\275\256", nullptr));
        monitor_enemy_pose_state->setText(QApplication::translate("DEBUG_PAGE", "\345\274\247\345\272\2461.6", nullptr));
        monitor_game_progress_title->setText(QApplication::translate("DEBUG_PAGE", "\346\257\224\350\265\233\347\212\266\346\200\201", nullptr));
        monitor_game_progress_state->setText(QApplication::translate("DEBUG_PAGE", "\350\277\233\350\241\214\344\270\255", nullptr));
        monitor_HP_deduction_reason_title->setText(QApplication::translate("DEBUG_PAGE", "\346\211\243\350\241\200\345\216\237\345\233\240", nullptr));
        monitor_HP_deduction_reason_state->setText(QApplication::translate("DEBUG_PAGE", "\350\243\205\347\224\262\346\235\277\345\217\227\345\207\273", nullptr));
        monitor_armor_id_title->setText(QApplication::translate("DEBUG_PAGE", "\345\217\227\345\207\273\350\243\205\347\224\262", nullptr));
        monitor_armor_id_state->setText(QApplication::translate("DEBUG_PAGE", "\347\274\226\345\217\267\351\233\266", nullptr));
        monitor_event_data_title->setText(QApplication::translate("DEBUG_PAGE", "\347\211\271\346\256\212\344\272\213\344\273\266", nullptr));
        monitor_event_data_state->setText(QApplication::translate("DEBUG_PAGE", "\346\227\240", nullptr));
        logQS_LOG_TEXT->setHtml(QApplication::translate("DEBUG_PAGE", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'\351\224\220\345\255\227\344\272\221\345\255\227\345\272\223\347\262\227\345\234\206GBK'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.597786068, 1591.491000000]: odom received!</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ WARN] [1736862284.599460312, 1591.492000000]: global_costmap: Pre-Hydro parameter &quot;static_map&quot; unused since &quot;plugins&quot; is provided</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px;"
                        " margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.600168732, 1591.493000000]: global_costmap: Using plugin &quot;static_layer&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.611879741, 1591.502000000]: Requesting the map...</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.816992482, 1591.707000000]: Resizing costmap to 599 X 301 at 0.050000 m/pix</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.917343816, 1591.806000000]: Received a 599 X 301 map at 0.050000 m/pix</span></p>\n"
"<p style=\" "
                        "margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.919314045, 1591.808000000]: global_costmap: Using plugin &quot;obstacle_layer&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.921515204, 1591.810000000]:     Subscribed to Topics: laser_scan_sensor</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.941431773, 1591.829000000]: global_costmap: Using plugin &quot;inflation_layer&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ WARN] [1736862284.979031800, 1591.866000000]: loc"
                        "al_costmap: Pre-Hydro parameter &quot;static_map&quot; unused since &quot;plugins&quot; is provided</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.979963551, 1591.866000000]: local_costmap: Using plugin &quot;static_layer&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.983346761, 1591.871000000]: Requesting the map...</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862284.984977924, 1591.872000000]: Resizing costmap to 599 X 301 at 0.050000 m/pix</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"
                        "\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862285.086585258, 1591.973000000]: Received a 599 X 301 map at 0.050000 m/pix</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862285.089201612, 1591.975000000]: local_costmap: Using plugin &quot;obstacle_layer&quot;</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862285.092764961, 1591.978000000]:     Subscribed to Topics: laser_scan_sensor</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862285.129995284, 1592.015000000]: Created local_planner base_local_planner/TrajectoryPlannerROS</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; "
                        "margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'SimSun';\">[ INFO] [1736862285.135387563, 1592.018000000]: Sim period is set to 0.05</span></p></body></html>", nullptr));
        Sim_Data_game_progress_title->setText(QApplication::translate("DEBUG_PAGE", "\346\257\224\350\265\233\347\212\266\346\200\201", nullptr));
        Sim_Data_game_progress_QL->setText(QApplication::translate("DEBUG_PAGE", "4", nullptr));
        Sim_Data_own_robot_HP_title->setText(QApplication::translate("DEBUG_PAGE", "\345\223\250\345\205\265\350\241\200\351\207\217", nullptr));
        Sim_Data_own_robot_HP_QL->setText(QApplication::translate("DEBUG_PAGE", "555", nullptr));
        Sim_Data_enemy_base_HP_title->setText(QApplication::translate("DEBUG_PAGE", "\346\225\214\346\226\271\345\237\272\345\234\260", nullptr));
        Sim_Data_enemy_base_HP_QL->setText(QApplication::translate("DEBUG_PAGE", "900", nullptr));
        Sim_Data_HP_deduction_reason_title->setText(QApplication::translate("DEBUG_PAGE", "\346\211\243\350\241\200\345\216\237\345\233\240", nullptr));
        Sim_Data_HP_deduction_reason_QL->setText(QApplication::translate("DEBUG_PAGE", "1", nullptr));
        Sim_Data_shoot_num_title->setText(QApplication::translate("DEBUG_PAGE", "\345\255\220\345\274\271\346\225\260\351\207\217", nullptr));
        Sim_Data_shoot_num_QL->setText(QApplication::translate("DEBUG_PAGE", "80", nullptr));
        Sim_Data_stage_remain_time_title->setText(QApplication::translate("DEBUG_PAGE", "\345\211\251\344\275\231\346\227\266\351\227\264", nullptr));
        Sim_Data_stage_remain_time_QL->setText(QApplication::translate("DEBUG_PAGE", "345", nullptr));
        Sim_Data_vision_status_title->setText(QApplication::translate("DEBUG_PAGE", "\350\247\206\350\247\211\347\212\266\346\200\201", nullptr));
        Sim_Data_vision_status_QL->setText(QApplication::translate("DEBUG_PAGE", "1", nullptr));
        Sim_Data_own_base_HP_title->setText(QApplication::translate("DEBUG_PAGE", "\345\267\261\346\226\271\345\237\272\345\234\260", nullptr));
        Sim_Data_own_base_HP_QL->setText(QApplication::translate("DEBUG_PAGE", "1400", nullptr));
        Sim_Data_event_data_title->setText(QApplication::translate("DEBUG_PAGE", "\347\211\271\346\256\212\344\272\213\344\273\266", nullptr));
        Sim_Data_event_data_QL->setText(QApplication::translate("DEBUG_PAGE", "0", nullptr));
        Sim_Data_armor_id_title->setText(QApplication::translate("DEBUG_PAGE", "\345\217\227\345\207\273\350\243\205\347\224\262", nullptr));
        Sim_Data_armor_id_QL->setText(QApplication::translate("DEBUG_PAGE", "0", nullptr));
        Sim_Data_TITLE->setText(QApplication::translate("DEBUG_PAGE", "\346\250\241\346\213\237\346\225\260\346\215\256\350\241\250", nullptr));
        Sim_Data_tips->setText(QApplication::translate("DEBUG_PAGE", "\350\257\264\346\230\216:\345\217\257\344\273\245\347\233\264\346\216\245\347\274\226\350\276\221\346\225\260\346\215\256\350\277\233\350\241\214\350\260\203\350\257\225", nullptr));
        Sim_Data_change_QP->setText(QApplication::translate("DEBUG_PAGE", "\347\241\256\350\256\244", nullptr));
        DEBUG_TITLE->setText(QApplication::translate("DEBUG_PAGE", "\345\206\263\347\255\226\350\260\203\350\257\225\346\216\247\345\210\266\345\217\260", nullptr));
        rules_date_title->setText(QApplication::translate("DEBUG_PAGE", "\350\247\204\345\210\231\346\227\245\346\234\237", nullptr));
        rules_date_state->setText(QApplication::translate("DEBUG_PAGE", "2025/1/15", nullptr));
        rules_version_title->setText(QApplication::translate("DEBUG_PAGE", "\350\247\204\345\210\231\347\211\210\346\234\254", nullptr));
        rules_version_state->setText(QApplication::translate("DEBUG_PAGE", "1.0.0", nullptr));
        rules_nums_title->setText(QApplication::translate("DEBUG_PAGE", "\345\206\263\347\255\226\346\225\260\351\207\217", nullptr));
        rules_nums_value->setText(QApplication::translate("DEBUG_PAGE", "37", nullptr));
        rules_path_QL->setText(QApplication::translate("DEBUG_PAGE", "/home/user/documents/rules/rule.json", nullptr));
        refresh_rules_QP->setText(QApplication::translate("DEBUG_PAGE", "\347\212\266\346\200\201\345\210\267\346\226\260", nullptr));
        reload_rules_QP->setText(QApplication::translate("DEBUG_PAGE", "\350\247\204\345\210\231\351\207\215\350\275\275", nullptr));
        rules_path_tips->setText(QApplication::translate("DEBUG_PAGE", "\350\257\264\346\230\216:\344\275\277\347\224\250\350\247\204\345\210\231\351\207\215\350\275\275\346\227\266\357\274\214\351\234\200\350\246\201\345\241\253\345\206\231\347\273\235\345\257\271\350\267\257\345\276\204", nullptr));
        goal_pose_title->setText(QApplication::translate("DEBUG_PAGE", "\350\210\252\347\202\271\350\260\203\350\257\225", nullptr));
        goal_pose_X->setText(QString());
        goal_pose_Y->setText(QString());
        goal_pose_YAW->setText(QString());
        goal_pose_tips->setText(QApplication::translate("DEBUG_PAGE", "\350\257\264\346\230\216:\345\210\206\345\210\253\345\257\271\345\272\224X\345\200\274\357\274\214Y\345\200\274 \344\270\216 YAW\345\201\217\350\275\254\350\247\222", nullptr));
        goal_pose_send_way_title->setText(QApplication::translate("DEBUG_PAGE", "\347\233\264\346\216\245\345\217\221\351\200\201", nullptr));
        goal_pose_send_QP->setText(QApplication::translate("DEBUG_PAGE", "\345\217\221\351\200\201", nullptr));
        goal_pose_send_way_tips->setText(QApplication::translate("DEBUG_PAGE", "\350\257\264\346\230\216:\351\273\230\350\256\244\345\205\210\350\275\254\345\217\221\347\273\231\345\206\263\347\255\226\350\212\202\347\202\271\344\275\234\344\270\272\346\234\200\351\253\230\344\274\230\345\205\210\347\272\247\344\273\273\345\212\241\357\274\214\350\213\245\346\230\257\347\233\264\346\216\245\345\217\221\351\200\201\347\233\264\346\216\245\345\217\221\351\200\201\345\257\274\350\210\252\350\257\267\346\261\202", nullptr));
        debug_send_title->setText(QApplication::translate("DEBUG_PAGE", "\345\217\221\351\200\201\346\250\241\346\213\237\346\225\260\346\215\256", nullptr));
        debug_listen_title->setText(QApplication::translate("DEBUG_PAGE", "\347\233\221\345\220\254\345\206\263\347\255\226\346\225\260\346\215\256", nullptr));
        pub_topic_QL->setText(QApplication::translate("DEBUG_PAGE", "/referee/msg", nullptr));
        datadebug_title->setText(QApplication::translate("DEBUG_PAGE", "\346\225\260\346\215\256\350\260\203\350\257\225", nullptr));
        debug_tips->setText(QApplication::translate("DEBUG_PAGE", "\350\257\264\346\230\216:\345\217\257\344\273\245\351\200\232\350\277\207\345\217\221\351\200\201\346\250\241\346\213\237\346\225\260\346\215\256\351\252\214\350\257\201\345\206\263\347\255\226\357\274\214\345\234\250\344\270\213\346\226\271\345\241\253\345\206\231\345\220\210\351\200\202\347\232\204\350\257\235\351\242\230\345\220\215\347\247\260", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DEBUG_PAGE: public Ui_DEBUG_PAGE {};
} // namespace Ui

QT_END_NAMESPACE

#endif // DEBUGPSXLZG_H
