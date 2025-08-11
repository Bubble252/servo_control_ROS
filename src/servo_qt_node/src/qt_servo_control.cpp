#include "servo_qt_node/qt_servo_control.h"
#include <QtCharts>
#include <QTimer>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <cmath>

QtServoControl::QtServoControl(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent), nh_(nh)
{
    // 发布舵机目标角度，消息格式: id + value (单位弧度*100)
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("servo_target", 10);

    // 订阅反馈
    fb_sub_ = nh_.subscribe("/feedback_feetech_joint_states", 100, &QtServoControl::feedbackCallback, this);

    // UI初始化
    label1_ = new QLabel("Servo 1 Angle (rad): 0.00");
    label2_ = new QLabel("Servo 2 Angle (rad): 0.00");

    slider1_ = new QSlider(Qt::Horizontal);
    slider1_->setRange(-314, 314);  // -π 到 π, *100缩放
    slider1_->setValue(0);

    slider2_ = new QSlider(Qt::Horizontal);
    slider2_->setRange(-314, 314);
    slider2_->setValue(0);

    connect(slider1_, &QSlider::valueChanged, this, &QtServoControl::onSlider1Changed);
    connect(slider2_, &QSlider::valueChanged, this, &QtServoControl::onSlider2Changed);

    // 初始化图表曲线
    series_pos_1_ = new QtCharts::QLineSeries();
    series_pos_1_->setName("ID1 Position");
    series_pos_2_ = new QtCharts::QLineSeries();
    series_pos_2_->setName("ID2 Position");

    series_speed_1_ = new QtCharts::QLineSeries();
    series_speed_1_->setName("ID1 Speed");
    series_speed_2_ = new QtCharts::QLineSeries();
    series_speed_2_->setName("ID2 Speed");

    series_current_1_ = new QtCharts::QLineSeries();
    series_current_1_->setName("ID1 Effort");
    series_current_2_ = new QtCharts::QLineSeries();
    series_current_2_->setName("ID2 Effort");

    // 位置图
    QtCharts::QChart* chart_pos = new QtCharts::QChart();
    chart_pos->addSeries(series_pos_1_);
    chart_pos->addSeries(series_pos_2_);

    axisX_pos_ = new QtCharts::QValueAxis();
    axisX_pos_->setRange(0, maxDataCount);
    axisX_pos_->setLabelFormat("%d");
    axisX_pos_->setTitleText("Sample");

    axisY_pos_ = new QtCharts::QValueAxis();
    axisY_pos_->setRange(-3.2, 3.2);
    axisY_pos_->setTitleText("Position (rad)");

    chart_pos->addAxis(axisX_pos_, Qt::AlignBottom);
    chart_pos->addAxis(axisY_pos_, Qt::AlignLeft);
    series_pos_1_->attachAxis(axisX_pos_);
    series_pos_1_->attachAxis(axisY_pos_);
    series_pos_2_->attachAxis(axisX_pos_);
    series_pos_2_->attachAxis(axisY_pos_);
    chart_pos->setTitle("Position");

    chartView_pos_ = new QtCharts::QChartView(chart_pos);

    // 速度图
    QtCharts::QChart* chart_speed = new QtCharts::QChart();
    chart_speed->addSeries(series_speed_1_);
    chart_speed->addSeries(series_speed_2_);

    axisX_speed_ = new QtCharts::QValueAxis();
    axisX_speed_->setRange(0, maxDataCount);
    axisX_speed_->setLabelFormat("%d");
    axisX_speed_->setTitleText("Sample");

    axisY_speed_ = new QtCharts::QValueAxis();
    axisY_speed_->setRange(-10, 10);
    axisY_speed_->setTitleText("Speed (rad/s)");

    chart_speed->addAxis(axisX_speed_, Qt::AlignBottom);
    chart_speed->addAxis(axisY_speed_, Qt::AlignLeft);
    series_speed_1_->attachAxis(axisX_speed_);
    series_speed_1_->attachAxis(axisY_speed_);
    series_speed_2_->attachAxis(axisX_speed_);
    series_speed_2_->attachAxis(axisY_speed_);
    chart_speed->setTitle("Speed");

    chartView_speed_ = new QtCharts::QChartView(chart_speed);

    // 力矩图（effort）
    QtCharts::QChart* chart_current = new QtCharts::QChart();
    chart_current->addSeries(series_current_1_);
    chart_current->addSeries(series_current_2_);

    axisX_current_ = new QtCharts::QValueAxis();
    axisX_current_->setRange(0, maxDataCount);
    axisX_current_->setLabelFormat("%d");
    axisX_current_->setTitleText("Sample");

    axisY_current_ = new QtCharts::QValueAxis();
    axisY_current_->setRange(0, 15);
    axisY_current_->setTitleText("Effort (Nm)");

    chart_current->addAxis(axisX_current_, Qt::AlignBottom);
    chart_current->addAxis(axisY_current_, Qt::AlignLeft);
    series_current_1_->attachAxis(axisX_current_);
    series_current_1_->attachAxis(axisY_current_);
    series_current_2_->attachAxis(axisX_current_);
    series_current_2_->attachAxis(axisY_current_);
    chart_current->setTitle("Effort");

    chartView_current_ = new QtCharts::QChartView(chart_current);

    // 主布局
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(label1_);
    mainLayout->addWidget(slider1_);
    mainLayout->addWidget(label2_);
    mainLayout->addWidget(slider2_);
    mainLayout->addWidget(chartView_pos_);
    mainLayout->addWidget(chartView_speed_);
    mainLayout->addWidget(chartView_current_);

    setLayout(mainLayout);

    // 定时器更新 ros spin 和图表
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &QtServoControl::rosSpin);
    connect(timer, &QTimer::timeout, this, &QtServoControl::updateCharts);
    timer->start(50);
}

QtServoControl::~QtServoControl() {}

void QtServoControl::onSlider1Changed(int value) {
    double rad = value / 100.0;
    label1_->setText(QString("Servo 1 Angle (rad): %1").arg(rad, 0, 'f', 2));
    std_msgs::Int32MultiArray msg;
    msg.data = {1, value};
    pub_.publish(msg);
}

void QtServoControl::onSlider2Changed(int value) {
    double rad = value / 100.0;
    label2_->setText(QString("Servo 2 Angle (rad): %1").arg(rad, 0, 'f', 2));
    std_msgs::Int32MultiArray msg;
    msg.data = {2, value};
    pub_.publish(msg);
}

void QtServoControl::rosSpin() {
    ros::spinOnce();
}

void QtServoControl::feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    for (size_t i = 0; i < 2; ++i) {  // 注意这里用 size_t
        int id = static_cast<int>(i) + 1;
        if (feedbackData_.find(id) == feedbackData_.end()) {
            feedbackData_[id] = FeedbackData();
        }
        auto& data = feedbackData_[id];

        if (data.pos.size() >= maxDataCount) {
            data.pos.pop_front();
            data.speed.pop_front();
            data.current.pop_front();
        }

        if (i < msg->position.size() && i < msg->velocity.size() && i < msg->effort.size()) {
            data.pos.push_back(msg->position[i]);
            data.speed.push_back(msg->velocity[i]);
            data.current.push_back(msg->effort[i]);
        }
    }
}


void QtServoControl::updateCharts() {
    series_pos_1_->clear();
    series_pos_2_->clear();
    series_speed_1_->clear();
    series_speed_2_->clear();
    series_current_1_->clear();
    series_current_2_->clear();

    auto addPoints = [](QtCharts::QLineSeries* series, const std::deque<double>& data) {
        for (size_t i = 0; i < data.size(); ++i) {
            series->append(i, data[i]);
        }
    };

    if (feedbackData_.count(1)) {
        addPoints(series_pos_1_, feedbackData_[1].pos);
        addPoints(series_speed_1_, feedbackData_[1].speed);
        addPoints(series_current_1_, feedbackData_[1].current);
    }
    if (feedbackData_.count(2)) {
        addPoints(series_pos_2_, feedbackData_[2].pos);
        addPoints(series_speed_2_, feedbackData_[2].speed);
        addPoints(series_current_2_, feedbackData_[2].current);
    }
}

