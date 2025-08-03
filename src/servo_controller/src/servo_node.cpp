#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>  // 假设订阅数组：[ID, target_angle, speed(optional), acc(optional)]
#include "servo_controller/servo.h"

// 全局Servo对象指针，实际应用中应管理多个舵机对象，示范只做一个简单例子
std::map<int, Servo*> servo_map;

void controlCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // 约定消息格式：[ID, target_angle, speed_max (opt), acceleration (opt)]
    if (msg->data.size() < 2) {
        ROS_WARN("消息长度不足，至少需要ID和目标角度");
        return;
    }
    int id = msg->data[0];
    int target_angle = msg->data[1];
    int speed_max = 1000;       // 默认值
    int acceleration = 50;      // 默认值
    if (msg->data.size() >= 3) speed_max = msg->data[2];
    if (msg->data.size() >= 4) acceleration = msg->data[3];

    // 查找是否已有对应ID的Servo对象，没有则新建
    if (servo_map.find(id) == servo_map.end()) {
        // 假设串口固定 /dev/ttyUSB0，Kp, Ki, Kd设成默认值0.1,0.01,0.05，实际可从参数服务器获取
        try {
            servo_map[id] = new Servo(id, "/dev/ttyUSB0", 0.1f, 0.01f, 0.05f);
            ROS_INFO("创建并初始化舵机ID=%d", id);
        } catch (const std::exception& e) {
            ROS_ERROR("初始化舵机ID=%d失败: %s", id, e.what());
            return;
        }
    }
    Servo* servo = servo_map[id];

    // 可选设置速度和加速度
    servo->setSpeedMax(speed_max);
    servo->setAcceleration(acceleration);

    // 调用PID角度控制（你的函数）
    servo->PID_setAngle_control(target_angle);

    // 获取并打印反馈
    auto fb = servo->get_feedback(id);
    if (fb.success) {
        ROS_INFO("舵机ID=%d 位置=%d 速度=%d 电流=%d 负载=%d 电压=%d",
                 id, fb.pos, fb.speed, fb.current, fb.load, fb.voltage);
    } else {
        ROS_WARN("获取舵机ID=%d反馈失败", id);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("servo_target", 10, controlCallback);

    ROS_INFO("舵机控制节点启动，等待目标角度命令...");

    ros::spin();

    // 程序退出前清理
    for (auto& pair : servo_map) {
        delete pair.second;
    }
    servo_map.clear();

    return 0;
}

