#ifndef QT_SERVO_CONTROL_H
#define QT_SERVO_CONTROL_H

#include <QWidget>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <deque>
#include <map>
#include <QLabel>
#include <QSlider>

struct FeedbackData {
    std::deque<double> pos;
    std::deque<double> speed;
    std::deque<double> current;
};

class QtServoControl : public QWidget
{
    Q_OBJECT
public:
    explicit QtServoControl(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~QtServoControl();

private slots:
    void onSlider1Changed(int value);
    void onSlider2Changed(int value);
    void rosSpin();
    void updateCharts();

private:

// 在 QtServoControl 类里 private 成员区添加：
QtCharts::QChartView* chartView_pos_;
QtCharts::QChartView* chartView_speed_;
QtCharts::QChartView* chartView_current_;

    void publishJointStateTargets();
    void feedbackCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Publisher pub_joint_state_;
    ros::Subscriber fb_sub_;

    QLabel* label1_;
    QLabel* label2_;
    QSlider* slider1_;
    QSlider* slider2_;

    QtCharts::QLineSeries* series_pos_1_;
    QtCharts::QLineSeries* series_pos_2_;
    QtCharts::QLineSeries* series_speed_1_;
    QtCharts::QLineSeries* series_speed_2_;
    QtCharts::QLineSeries* series_current_1_;
    QtCharts::QLineSeries* series_current_2_;

    QtCharts::QValueAxis* axisX_pos_;
    QtCharts::QValueAxis* axisY_pos_;
    QtCharts::QValueAxis* axisX_speed_;
    QtCharts::QValueAxis* axisY_speed_;
    QtCharts::QValueAxis* axisX_current_;
    QtCharts::QValueAxis* axisY_current_;

    double target_angle_1_ = 0.0;
    double target_angle_2_ = 0.0;

    static constexpr int maxDataCount = 100;

    std::map<int, FeedbackData> feedbackData_;
};

#endif // QT_SERVO_CONTROL_H

