#include "servo_qt_node/qt_servo_control.h"
#include <QtCharts>
#include <QTimer>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>  // 加这个用于ROS日志

QtServoControl::QtServoControl(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent), nh_(nh)
{
    ROS_INFO("QtServoControl constructor called");

    // 发布舵机目标角度
    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("/command_joint_states", 10);

    // 订阅反馈
    fb_sub_ = nh_.subscribe("/feedback_feetech_joint_states", 100, &QtServoControl::feedbackCallback, this);

    // UI初始化
    label1_ = new QLabel("Servo 1 Angle (rad): 0.00");
    label2_ = new QLabel("Servo 2 Angle (rad): 0.00");

    slider1_ = new QSlider(Qt::Horizontal);
    slider1_->setRange(-314, 314);  // -π 到 π, *100缩放
    slider1_->setValue(-268);

    slider2_ = new QSlider(Qt::Horizontal);
    slider2_->setRange(-314, 314);
    slider2_->setValue(-16);

    connect(slider1_, &QSlider::valueChanged, this, &QtServoControl::onSlider1Changed);
    connect(slider2_, &QSlider::valueChanged, this, &QtServoControl::onSlider2Changed);

    // 图表初始化部分省略，为节省篇幅...

    // 定时器更新 ros spin 和图表
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &QtServoControl::rosSpin);
    connect(timer, &QTimer::timeout, this, &QtServoControl::updateCharts);
    timer->start(50);

    ROS_INFO("QtServoControl initialized and timer started");
}

QtServoControl::~QtServoControl() {
    ROS_INFO("QtServoControl destructor called");
}

void QtServoControl::onSlider1Changed(int value) {
    target_angle_1_ = value / 100.0;
    label1_->setText(QString("Servo 1 Angle (rad): %1").arg(target_angle_1_, 0, 'f', 2));
    ROS_INFO("Slider1 changed: target_angle_1_ = %f", target_angle_1_);
    publishJointStateTargets();
}

void QtServoControl::onSlider2Changed(int value) {
    target_angle_2_ = value / 100.0;
    label2_->setText(QString("Servo 2 Angle (rad): %1").arg(target_angle_2_, 0, 'f', 2));
    ROS_INFO("Slider2 changed: target_angle_2_ = %f", target_angle_2_);
    publishJointStateTargets();
}

void QtServoControl::publishJointStateTargets() {
    sensor_msgs::JointState msg;
    msg.name.resize(21);
    msg.position.resize(21);

    // 清零
    for (int i = 0; i < 21; ++i) {
        msg.name[i] = "";
        msg.position[i] = 0.0;
    }

    msg.name[10] = "servo_1";
    msg.name[11] = "servo_2";

    //msg.position[10] = target_angle_1_;
    msg.position[11] = target_angle_2_;

    pub_joint_state_.publish(msg);

    ROS_INFO("Published JointState targets: servo_1=%.3f rad, servo_2=%.3f rad", target_angle_1_, target_angle_2_);
}

void QtServoControl::rosSpin() {
    ros::spinOnce();
    ROS_DEBUG("Called ros::spinOnce()");
}

void QtServoControl::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("Received feedback message with %lu positions", msg->position.size());
    for (size_t i = 0; i < 2; ++i) {
        int id = static_cast<int>(i) + 1;
        if (feedbackData_.find(id) == feedbackData_.end()) {
            feedbackData_[id] = FeedbackData();
        }
        auto& data = feedbackData_[id];

        if (data.pos.size() >= maxDataCount) {
            data.pos.pop_front();
            data.speed.pop_front();
            data.effort.pop_front();
        }

        if (i < msg->position.size() && i < msg->velocity.size() && i < msg->effort.size()) {
            data.pos.push_back(msg->position[i]);
            data.speed.push_back(msg->velocity[i]);
            data.effort.push_back(msg->effort[i]);

            ROS_DEBUG("Feedback for ID=%d: pos=%.3f, speed=%.3f, effort=%.3f", id, msg->position[i], msg->velocity[i], msg->effort[i]);
        } else {
            ROS_WARN("Feedback message too short for ID=%d", id);
        }
    }
}

void QtServoControl::updateCharts() {
    series_pos_1_->clear();
    series_pos_2_->clear();
    series_speed_1_->clear();
    series_speed_2_->clear();
    series_effort_1_->clear();
    series_effort_2_->clear();

    auto addPoints = [](QtCharts::QLineSeries* series, const std::deque<double>& data) {
        for (size_t i = 0; i < data.size(); ++i) {
            series->append(i, data[i]);
        }
    };

    if (feedbackData_.count(1)) {
        addPoints(series_pos_1_, feedbackData_[1].pos);
        addPoints(series_speed_1_, feedbackData_[1].speed);
        addPoints(series_effort_1_, feedbackData_[1].effort);
        ROS_DEBUG("Updated charts for servo ID=1");
    }
    if (feedbackData_.count(2)) {
        addPoints(series_pos_2_, feedbackData_[2].pos);
        addPoints(series_speed_2_, feedbackData_[2].speed);
        addPoints(series_effort_2_, feedbackData_[2].effort);
        ROS_DEBUG("Updated charts for servo ID=2");
    }
}

