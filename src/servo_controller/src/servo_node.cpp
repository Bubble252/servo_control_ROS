
#include "servo_controller/ServoFeedback.h"  // 新增
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "servo_controller/servo.h"

// 舵机最大力矩列表，单位 Nm（请根据实际情况填写）
double servo_max_torque[10] = {
    10.3, 10.3, 10.3, 10.3, 10.3, 10.3, 10.3, 10.3, 10.3, 10.3
};

std::map<int, Servo*> servo_map;

// 弧度 -> 编码器值
inline int rad2enc(double rad) {
    return static_cast<int>(rad / 3.14 * 2048.0 + 2048.0);
}

// 编码器值 -> 弧度
inline double enc2rad(int enc) {
    return (enc - 2048) * 3.14 / 2048.0;
}

// 舵机 ID 映射 (JointState 索引 11~20 → 舵机 ID 1~10)
int jointIndexToServoId(int idx) {
    return idx - 10; // 11->1, 12->2, ..., 20->10
}

void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() < 21) {
        ROS_WARN("JointState.position 长度不足 21");
        return;
    }

    for (int idx = 11; idx <= 20; ++idx) {
        int servo_id = jointIndexToServoId(idx);
        double target_rad = msg->position[idx];
        int target_enc = rad2enc(target_rad);

        if (servo_map.find(servo_id) == servo_map.end()) {
            try {
                servo_map[servo_id] = new Servo(servo_id, "/dev/ttyUSB0", 0.1f, 0.01f, 0.05f);
                ROS_INFO("创建并初始化舵机 ID=%d", servo_id);
            } catch (const std::exception& e) {
                ROS_ERROR("初始化舵机 ID=%d 失败: %s", servo_id, e.what());
                continue;
            }
        }

        Servo* servo = servo_map[servo_id];
        servo->setSpeedMax(1000);
        servo->setAcceleration(50);
        servo->PID_setAngle_control(target_enc);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/command_joint_states", 10, commandCallback);
    ros::Publisher fb_pub = nh.advertise<sensor_msgs::JointState>("/feedback_feetech_joint_states", 10);

    ros::Rate rate(50); // 50Hz 反馈频率
    while (ros::ok()) {
        ros::spinOnce();

        sensor_msgs::JointState fb_msg;
        fb_msg.header.stamp = ros::Time::now();
        fb_msg.name.resize(10);
        fb_msg.position.resize(10);
        fb_msg.velocity.resize(10);
        fb_msg.effort.resize(10);

        for (int i = 0; i < 10; ++i) {
            int servo_id = i + 1; // ID 1~10

            fb_msg.name[i] = "servo_" + std::to_string(servo_id);

            if (servo_map.find(servo_id) == servo_map.end()) {
                // 如果没初始化，发0
                fb_msg.position[i] = 0.0;
                fb_msg.velocity[i] = 0.0;
                fb_msg.effort[i] = 0.0;
                continue;
            }

            auto fb = servo_map[servo_id]->get_feedback(servo_id);

            // 位置转弧度
            fb_msg.position[i] = enc2rad(fb.pos);

            // 速度转弧度/秒 (假设speed单位与pos相同)
            fb_msg.velocity[i] = enc2rad(fb.speed);

            // 力矩转换，fb.load单位是0.1%最大力矩，转换为 Nm
            fb_msg.effort[i] = (static_cast<double>(fb.load) / 1000.0) * servo_max_torque[i];
        }

        fb_pub.publish(fb_msg);
        rate.sleep();
    }

    for (auto& pair : servo_map) {
        delete pair.second;
    }
    servo_map.clear();

    return 0;
}

