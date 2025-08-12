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

    // 发布和订阅
    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>("/command_joint_states", 10);
    fb_sub_ = nh_.subscribe("/feedback_feetech_joint_states", 100, &QtServoControl::feedbackCallback, this);

    // UI初始化：标签和滑块
    label1_ = new QLabel("Servo 1 Angle (rad): 0.00");
    label2_ = new QLabel("Servo 2 Angle (rad): 0.00");

    slider1_ = new QSlider(Qt::Horizontal);
    slider1_->setRange(-314, 314);
    slider1_->setValue(-268);
    slider2_ = new QSlider(Qt::Horizontal);
    slider2_->setRange(-314, 314);
    slider2_->setValue(-16);

    connect(slider1_, &QSlider::valueChanged, this, &QtServoControl::onSlider1Changed);
    connect(slider2_, &QSlider::valueChanged, this, &QtServoControl::onSlider2Changed);

    // **图表系列初始化**
    series_pos_1_ = new QtCharts::QLineSeries(this);
    series_pos_2_ = new QtCharts::QLineSeries(this);
    series_speed_1_ = new QtCharts::QLineSeries(this);
    series_speed_2_ = new QtCharts::QLineSeries(this);
    series_effort_1_ = new QtCharts::QLineSeries(this);
    series_effort_2_ = new QtCharts::QLineSeries(this);

    // **创建图表及坐标轴**
    // 位置图表
    QtCharts::QChart* chart_pos = new QtCharts::QChart();
    chart_pos->addSeries(series_pos_1_);
    chart_pos->addSeries(series_pos_2_);
    axisX_pos_ = new QtCharts::QValueAxis();
    axisY_pos_ = new QtCharts::QValueAxis();
    chart_pos->addAxis(axisX_pos_, Qt::AlignBottom);
    chart_pos->addAxis(axisY_pos_, Qt::AlignLeft);
    series_pos_1_->attachAxis(axisX_pos_);
    series_pos_1_->attachAxis(axisY_pos_);
    series_pos_2_->attachAxis(axisX_pos_);
    series_pos_2_->attachAxis(axisY_pos_);
    chart_pos->setTitle("Position Feedback");

    // 速度图表
    QtCharts::QChart* chart_speed = new QtCharts::QChart();
    chart_speed->addSeries(series_speed_1_);
    chart_speed->addSeries(series_speed_2_);
    axisX_speed_ = new QtCharts::QValueAxis();
    axisY_speed_ = new QtCharts::QValueAxis();
    chart_speed->addAxis(axisX_speed_, Qt::AlignBottom);
    chart_speed->addAxis(axisY_speed_, Qt::AlignLeft);
    series_speed_1_->attachAxis(axisX_speed_);
    series_speed_1_->attachAxis(axisY_speed_);
    series_speed_2_->attachAxis(axisX_speed_);
    series_speed_2_->attachAxis(axisY_speed_);
    chart_speed->setTitle("Speed Feedback");

    // 力矩图表
    QtCharts::QChart* chart_effort = new QtCharts::QChart();
    chart_effort->addSeries(series_effort_1_);
    chart_effort->addSeries(series_effort_2_);
    axisX_effort_ = new QtCharts::QValueAxis();
    axisY_effort_ = new QtCharts::QValueAxis();
    chart_effort->addAxis(axisX_effort_, Qt::AlignBottom);
    chart_effort->addAxis(axisY_effort_, Qt::AlignLeft);
    series_effort_1_->attachAxis(axisX_effort_);
    series_effort_1_->attachAxis(axisY_effort_);
    series_effort_2_->attachAxis(axisX_effort_);
    series_effort_2_->attachAxis(axisY_effort_);
    chart_effort->setTitle("Effort Feedback");

    // **创建 ChartView 并开启抗锯齿**
    chartView_pos_ = new QtCharts::QChartView(chart_pos);
    chartView_pos_->setRenderHint(QPainter::Antialiasing);

    chartView_speed_ = new QtCharts::QChartView(chart_speed);
    chartView_speed_->setRenderHint(QPainter::Antialiasing);

    chartView_effort_ = new QtCharts::QChartView(chart_effort);
    chartView_effort_->setRenderHint(QPainter::Antialiasing);

    // **布局所有控件**
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->addWidget(label1_);
    layout->addWidget(slider1_);
    layout->addWidget(label2_);
    layout->addWidget(slider2_);
    layout->addWidget(chartView_pos_);
    layout->addWidget(chartView_speed_);
    layout->addWidget(chartView_effort_);
    setLayout(layout);

    // 定时器用于 rosSpin() 和图表更新
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

