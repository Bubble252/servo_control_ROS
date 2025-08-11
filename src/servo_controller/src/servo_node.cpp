#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "servo_controller/servo.h"
#include "servo_controller/ServoFeedback.h"  // 新增

std::map<int, Servo*> servo_map;
ros::Publisher fb_pub;

void controlCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) {
        ROS_WARN("消息长度不足，至少需要ID和目标角度");
        return;
    }
    int id = msg->data[0];
    int target_angle = msg->data[1];
    int speed_max = 1000;
    int acceleration = 50;
    if (msg->data.size() >= 3) speed_max = msg->data[2];
    if (msg->data.size() >= 4) acceleration = msg->data[3];

    if (servo_map.find(id) == servo_map.end()) {
        try {
            servo_map[id] = new Servo(id, "/dev/ttyUSB0", 0.1f, 0.01f, 0.05f);
            ROS_INFO("创建并初始化舵机ID=%d", id);
        } catch (const std::exception& e) {
            ROS_ERROR("初始化舵机ID=%d失败: %s", id, e.what());
            return;
        }
    }
    Servo* servo = servo_map[id];
    servo->setSpeedMax(speed_max);
    servo->setAcceleration(acceleration);
    servo->PID_setAngle_control(target_angle);

    auto fb = servo->get_feedback(id);

    // 发布反馈消息
    servo_controller::ServoFeedback fb_msg;
    fb_msg.id = id;
    fb_msg.pos = fb.pos;
    fb_msg.speed = fb.speed;
    fb_msg.current = fb.current;
    fb_msg.load = fb.load;
    fb_msg.voltage = fb.voltage;
    fb_msg.temper = fb.temper;    // 你消息里有温度
    fb_msg.move = fb.move;
    fb_msg.success = fb.success;

    fb_pub.publish(fb_msg);

    if (fb.success) {
        ROS_INFO("舵机ID=%d 位置=%d 速度=%d 电流=%d 负载=%d 电压=%d 温度=%d",
                 id, fb.pos, fb.speed, fb.current, fb.load, fb.voltage, fb.temper);
    } else {
        ROS_WARN("获取舵机ID=%d反馈失败", id);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh;

    fb_pub = nh.advertise<servo_controller::ServoFeedback>("servo_feedback", 10);
    ros::Subscriber sub = nh.subscribe("servo_target", 10, controlCallback);

    ROS_INFO("舵机控制节点启动，等待目标角度命令...");

    ros::spin();

    for (auto& pair : servo_map) {
        delete pair.second;
    }
    servo_map.clear();

    return 0;
}

