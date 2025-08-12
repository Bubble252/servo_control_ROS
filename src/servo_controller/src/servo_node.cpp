#include "servo_controller/ServoFeedback.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "servo_controller/servo.h"
#include <map>
#include <sstream>
#include <iomanip>

#include <string>

// 每个舵机的最大扭矩（单位：Nm），用于反馈计算
double servo_max_torque[10] = {10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3};

// 保存舵机对象的map： key=舵机ID, value=Servo类指针
std::map<int, Servo*> servo_map;

// 保存目标位置的map： key=舵机ID, value=目标编码器值
std::map<int, int> target_positions;  

// 弧度转编码器值
inline int rad2enc(double rad) {
    return static_cast<int>(rad / 3.14 * 2048.0 + 2048.0);
}

// 编码器值转弧度
inline double enc2rad(int enc) {
    return (enc - 2048) * 3.14 / 2048.0;
}

// 将JointState消息中的关节索引转成舵机ID
// 例如 JointState[11] -> 舵机ID 1
int jointIndexToServoId(int idx) {
    return idx - 10; 
}

// ------------------- 接收控制指令的回调 -------------------
// 当有新的 /command_joint_states 消息时，这个函数会被调用
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("Received control command, position size=%lu", msg->position.size());

    // 如果指令长度不够，就直接忽略
    if (msg->position.size() < 21) {
        ROS_WARN("JointState.position length less than 21");
        return;
    }

    // 遍历关节索引 11~20（即舵机 1~10）
    for (int idx = 11; idx <= 20; ++idx) {
        int servo_id = jointIndexToServoId(idx);
        double target_rad = msg->position[idx];    // 目标弧度
        int target_enc = rad2enc(target_rad);      // 转成编码器值

        // 更新目标位置（不会立即发给舵机）
        target_positions[servo_id] = target_enc;

        ROS_DEBUG("Updated target for servo ID=%d: angle(rad)=%.3f, encoder=%d", 
                  servo_id, target_rad, target_enc);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("servo_controller_node started");

    // ------------------- 舵机初始化 -------------------
    for (int id = 1; id <= 10; ++id) {
        try {
            // 创建Servo对象，连接串口 /dev/ttyUSB0
            servo_map[id] = new Servo(id, "/dev/ttyUSB0", 0.7f, 0.0f, 0.0f);
            servo_map[id]->setSpeedMax(1000);   // 最大速度
            servo_map[id]->setAcceleration(50); // 加速度

            // 读取当前舵机位置，作为初始目标，防止启动时突然跳动
            auto fb = servo_map[id]->get_feedback(id);
            target_positions[id] = fb.pos;

            ROS_INFO("Initialized servo ID=%d on /dev/ttyUSB0", id);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize servo ID=%d: %s", id, e.what());
        }
    }

    // ------------------- ROS 话题 -------------------
    // 订阅控制指令话题（外部控制节点发布到这里）
    ros::Subscriber sub = nh.subscribe("/command_joint_states", 10, commandCallback);

    // 发布舵机反馈话题
    ros::Publisher fb_pub = nh.advertise<sensor_msgs::JointState>("/feedback_feetech_joint_states", 10);

    ros::Rate rate(100); // 循环频率 100Hz
    
    // 在main函数的主循环前定义计时变量
	ros::Time last_cycle_time = ros::Time::now();
	static int cycle_count = 0;  // 用于累计周期数
	static double total_cycle_time = 0.0;  // 累计总周期时间

// ------------------- 主循环 -------------------
while (ros::ok()) {
    ros::spinOnce();
    
    
    
    
    
        // --------------- 新增：计算循环频率 ---------------
    ros::Time current_time = ros::Time::now();
    ros::Duration cycle_duration = current_time - last_cycle_time;
    last_cycle_time = current_time;

    // 累计周期时间，每3个周期计算一次平均频率（减少打印开销）
    total_cycle_time += cycle_duration.toSec();
    cycle_count++;
    if (cycle_count >= 3) {
        double avg_period = total_cycle_time / cycle_count;
        double avg_freq = 1.0 / avg_period;
        ROS_INFO("Average control frequency: %.2f Hz (period: %.3f ms)", 
                 avg_freq, avg_period * 1000);
        cycle_count = 0;
        total_cycle_time = 0.0;
    }
    // --------------------------------------------------
    
    
    
    
    
    
    

    // 每秒打印一次 target_positions（编码器值和弧度）
    static double last_print_time = 0.0;
    double now_time = ros::Time::now().toSec();
    if (now_time - last_print_time >= 1.0) {
        std::ostringstream oss_enc, oss_rad;
        oss_enc << "[Target Enc] ";
        oss_rad << "[Target Rad] ";
        for (int id = 1; id <= 10; ++id) {
            int enc_val = target_positions[id];
            double rad_val = enc2rad(enc_val);
            oss_enc << enc_val << "\t";
            oss_rad << std::fixed << std::setprecision(3) << rad_val << "\t";
        }
        ROS_INFO_STREAM(oss_enc.str());
        ROS_INFO_STREAM(oss_rad.str());
        last_print_time = now_time;
    }

    // ----------- 发送控制命令 -----------
    for (auto& [id, servo] : servo_map) {
        if (!servo) {
            ROS_WARN("servo pointer for ID=%d is nullptr, skipping command", id);
            continue;
        }
        auto it = target_positions.find(id);
        if (it == target_positions.end()) {
            ROS_WARN("No target position found for servo ID=%d, skipping command", id);
            continue;
        }
        int target_enc = it->second;

        try {
            servo->PID_setAngle_control(target_enc);
        } catch (const std::exception& e) {
            ROS_ERROR("Exception when sending command to servo ID=%d: %s", id, e.what());
        } catch (...) {
            ROS_ERROR("Unknown exception when sending command to servo ID=%d", id);
        }
    }


        // ----------- 采集反馈并发布 -----------
        sensor_msgs::JointState fb_msg;
        fb_msg.header.stamp = ros::Time::now();
        fb_msg.name.resize(10);
        fb_msg.position.resize(10);
        fb_msg.velocity.resize(10);
        fb_msg.effort.resize(10);

        for (int i = 0; i < 10; ++i) {
            int servo_id = i + 1;
            fb_msg.name[i] = "servo_" + std::to_string(servo_id);

            auto it = servo_map.find(servo_id);
            if (it == servo_map.end() || !(it->second)) {
                fb_msg.position[i] = 0.0;
                fb_msg.velocity[i] = 0.0;
                fb_msg.effort[i] = 0.0;
                ROS_WARN_THROTTLE(10, "Servo ID=%d not initialized or null pointer, sending zero feedback", servo_id);
                continue;
            }

            try {
                auto fb = it->second->get_feedback(servo_id);
                fb_msg.position[i] = enc2rad(fb.pos);
                fb_msg.velocity[i] = enc2rad(fb.speed);
                fb_msg.effort[i] = (static_cast<double>(fb.load) / 1000.0) * servo_max_torque[i];
            } catch (const std::exception& e) {
                ROS_ERROR("Exception when getting feedback from servo ID=%d: %s", servo_id, e.what());
                fb_msg.position[i] = 0.0;
                fb_msg.velocity[i] = 0.0;
                fb_msg.effort[i] = 0.0;
            } catch (...) {
                ROS_ERROR("Unknown exception when getting feedback from servo ID=%d", servo_id);
                fb_msg.position[i] = 0.0;
                fb_msg.velocity[i] = 0.0;
                fb_msg.effort[i] = 0.0;
            }
        }

        // 发布反馈
        fb_pub.publish(fb_msg);

        rate.sleep();
    }

    // ------------------- 程序退出清理 -------------------
    ROS_INFO("Exiting and cleaning up");
    for (auto& pair : servo_map) {
        delete pair.second;
    }
    servo_map.clear();

    return 0;
}

