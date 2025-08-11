#include "servo_qt_node/qt_servo_control.h"
#include <QtCharts>
#include <boost/bind.hpp>  // 添加这个头文件（放在顶部）




void QtServoControl::feedbackCallback(const servo_controller::ServoFeedback::ConstPtr& msg) {
    int id = msg->id;
    if (feedbackData_.find(id) == feedbackData_.end()) {
        feedbackData_[id] = FeedbackData();
    }
    auto& data = feedbackData_[id];

    if (data.pos.size() >= maxDataCount) {
        data.pos.pop_front();
        data.speed.pop_front();
        data.current.pop_front();
    }
    data.pos.append(msg->pos);
    data.speed.append(msg->speed);
    data.current.append(msg->current);
}

QtServoControl::~QtServoControl() {}

QtServoControl::QtServoControl(ros::NodeHandle& nh, QWidget* parent)
    : QWidget(parent), nh_(nh)
{
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("servo_target", 10);
    fb_sub_ = nh_.subscribe("servo_feedback", 100, &QtServoControl::feedbackCallback, this);
    




    // UI初始化
    slider1_ = new QSlider(Qt::Horizontal);
    slider1_->setRange(0, 180);
    slider1_->setValue(90);

    slider2_ = new QSlider(Qt::Horizontal);
    slider2_->setRange(0, 180);
    slider2_->setValue(90);

    label1_ = new QLabel("Servo 1 Angle: 90");
    label2_ = new QLabel("Servo 2 Angle: 90");

    connect(slider1_, &QSlider::valueChanged, this, &QtServoControl::onSlider1Changed);
    connect(slider2_, &QSlider::valueChanged, this, &QtServoControl::onSlider2Changed);

    // 初始化图表和曲线
    series_pos_1_ = new QLineSeries();
    series_pos_1_->setName("ID1 Position");
    series_pos_2_ = new QLineSeries();
    series_pos_2_->setName("ID2 Position");

    series_speed_1_ = new QLineSeries();
    series_speed_1_->setName("ID1 Speed");
    series_speed_2_ = new QLineSeries();
    series_speed_2_->setName("ID2 Speed");

    series_current_1_ = new QLineSeries();
    series_current_1_->setName("ID1 Current");
    series_current_2_ = new QLineSeries();
    series_current_2_->setName("ID2 Current");

    // 位置图
    QChart* chart_pos = new QChart();
    chart_pos->addSeries(series_pos_1_);
    chart_pos->addSeries(series_pos_2_);
    axisX_pos_ = new QValueAxis();
    axisX_pos_->setRange(0, maxDataCount);
    axisX_pos_->setLabelFormat("%d");
    axisX_pos_->setTitleText("Sample");
    axisY_pos_ = new QValueAxis();
    axisY_pos_->setRange(0, 180);
    axisY_pos_->setTitleText("Position");
    chart_pos->addAxis(axisX_pos_, Qt::AlignBottom);
    chart_pos->addAxis(axisY_pos_, Qt::AlignLeft);
    series_pos_1_->attachAxis(axisX_pos_);
    series_pos_1_->attachAxis(axisY_pos_);
    series_pos_2_->attachAxis(axisX_pos_);
    series_pos_2_->attachAxis(axisY_pos_);
    chart_pos->setTitle("Position");

    chartView_pos_ = new QChartView(chart_pos);

    // 速度图
    QChart* chart_speed = new QChart();
    chart_speed->addSeries(series_speed_1_);
    chart_speed->addSeries(series_speed_2_);
    axisX_speed_ = new QValueAxis();
    axisX_speed_->setRange(0, maxDataCount);
    axisX_speed_->setLabelFormat("%d");
    axisX_speed_->setTitleText("Sample");
    axisY_speed_ = new QValueAxis();
    axisY_speed_->setRange(-1000, 1000);
    axisY_speed_->setTitleText("Speed");
    chart_speed->addAxis(axisX_speed_, Qt::AlignBottom);
    chart_speed->addAxis(axisY_speed_, Qt::AlignLeft);
    series_speed_1_->attachAxis(axisX_speed_);
    series_speed_1_->attachAxis(axisY_speed_);
    series_speed_2_->attachAxis(axisX_speed_);
    series_speed_2_->attachAxis(axisY_speed_);
    chart_speed->setTitle("Speed");

    chartView_speed_ = new QChartView(chart_speed);

    // 电流图
    QChart* chart_current = new QChart();
    chart_current->addSeries(series_current_1_);
    chart_current->addSeries(series_current_2_);
    axisX_current_ = new QValueAxis();
    axisX_current_->setRange(0, maxDataCount);
    axisX_current_->setLabelFormat("%d");
    axisX_current_->setTitleText("Sample");
    axisY_current_ = new QValueAxis();
    axisY_current_->setRange(0, 1000);
    axisY_current_->setTitleText("Current");
    chart_current->addAxis(axisX_current_, Qt::AlignBottom);
    chart_current->addAxis(axisY_current_, Qt::AlignLeft);
    series_current_1_->attachAxis(axisX_current_);
    series_current_1_->attachAxis(axisY_current_);
    series_current_2_->attachAxis(axisX_current_);
    series_current_2_->attachAxis(axisY_current_);
    chart_current->setTitle("Current");

    chartView_current_ = new QChartView(chart_current);

    // 布局
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(label1_);
    mainLayout->addWidget(slider1_);
    mainLayout->addWidget(label2_);
    mainLayout->addWidget(slider2_);
    mainLayout->addWidget(chartView_pos_);
    mainLayout->addWidget(chartView_speed_);
    mainLayout->addWidget(chartView_current_);

    setLayout(mainLayout);

    // 定时器，定期调用ros::spinOnce和更新界面
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &QtServoControl::rosSpin);
    connect(timer, &QTimer::timeout, this, &QtServoControl::updateCharts);
    timer->start(50);  // 20Hz刷新
}

void QtServoControl::onSlider1Changed(int value) {
    label1_->setText(QString("Servo 1 Angle: %1").arg(value));
    std_msgs::Int32MultiArray msg;
    msg.data = {1, value};
    pub_.publish(msg);
}

void QtServoControl::onSlider2Changed(int value) {
    label2_->setText(QString("Servo 2 Angle: %1").arg(value));
    std_msgs::Int32MultiArray msg;
    msg.data = {2, value};
    pub_.publish(msg);
}

void QtServoControl::rosSpin() {
    ros::spinOnce();
}

// 接收反馈消息回调


void QtServoControl::updateCharts() {
    // 清除旧数据
    series_pos_1_->clear();
    series_pos_2_->clear();
    series_speed_1_->clear();
    series_speed_2_->clear();
    series_current_1_->clear();
    series_current_2_->clear();

    auto addPoints = [](QLineSeries* series, const QVector<double>& data) {
        for (int i = 0; i < data.size(); ++i) {
            series->append(i, data[i]);
        }
    };

    // ID=1数据
    if (feedbackData_.count(1)) {
        addPoints(series_pos_1_, feedbackData_[1].pos);
        addPoints(series_speed_1_, feedbackData_[1].speed);
        addPoints(series_current_1_, feedbackData_[1].current);
    }
    // ID=2数据
    if (feedbackData_.count(2)) {
        addPoints(series_pos_2_, feedbackData_[2].pos);
        addPoints(series_speed_2_, feedbackData_[2].speed);
        addPoints(series_current_2_, feedbackData_[2].current);
    }
}

