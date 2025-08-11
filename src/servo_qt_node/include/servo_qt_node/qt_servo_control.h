#ifndef QT_SERVO_CONTROL_H
#define QT_SERVO_CONTROL_H

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QTimer>

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <servo_controller/ServoFeedback.h>  // 自定义反馈消息

#include <QVector>
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>

QT_CHARTS_USE_NAMESPACE

class QtServoControl : public QWidget {
    Q_OBJECT
public:
    explicit QtServoControl(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~QtServoControl();
private slots:
    void onSlider1Changed(int value);
    void onSlider2Changed(int value);
    void rosSpin();

    void updateCharts();
    void feedbackCallback(const servo_controller::ServoFeedback::ConstPtr& msg);

    

private:
    // UI
    QSlider* slider1_;
    QSlider* slider2_;
    QLabel* label1_;
    QLabel* label2_;

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber fb_sub_;

    // 实时数据缓存，简单起见限制数据点数，比如100点
    static const int maxDataCount = 100;

    // 分ID存储历史数据
    struct FeedbackData {
        QVector<double> pos;
        QVector<double> speed;
        QVector<double> current;
    };
    std::map<int, FeedbackData> feedbackData_;

    // 图表控件
    QChartView* chartView_pos_;
    QChartView* chartView_speed_;
    QChartView* chartView_current_;

    QLineSeries* series_pos_1_;
    QLineSeries* series_pos_2_;
    QLineSeries* series_speed_1_;
    QLineSeries* series_speed_2_;
    QLineSeries* series_current_1_;
    QLineSeries* series_current_2_;

    QValueAxis* axisX_pos_;
    QValueAxis* axisY_pos_;
    QValueAxis* axisX_speed_;
    QValueAxis* axisY_speed_;
    QValueAxis* axisX_current_;
    QValueAxis* axisY_current_;
};

#endif // QT_SERVO_CONTROL_H

