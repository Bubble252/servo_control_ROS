#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "SCServo.h" // 底层库（请确保库路径与 include 配置正确）
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <unistd.h> // usleep
#include <cmath>

using std::map;
using std::vector;
using std::string;

// -------------------- 配置常量 --------------------
const vector<uint8_t> servo_IDs = {1,2,3,4,5,6,7,8,9,10};
const int SERVO_ENCODER_RES = 4096; // 编码器分辨率 (0..4095)
double servo_max_torque[10] = {10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3};

SMS_STS sm_st; // 全局串口 / SCServo 对象

map<int,int> target_positions; // 目标位置 map<id, encoder>

// 速度/加速度 与 调速参数
const int SPEED_MAX = 2000;    // 最大速度（编码器单位）
const int ACCELERATION = 50;   // 加速度（示例）
const int ERROR_THRESHOLD = 10;// 误差阈值：小于此认为到位（编码器单位）
const int MIN_SPEED = 400;      // 最小速度（避免太小无力）


// -------------------- 内部反馈结构（带 success 标志） --------------------
// 为了在内存里保存是否读取成功，加 success 字段，而不依赖 ROS msg 的结构
struct InternalFeedback {
    int pos = 0;    // 位置 （编码器）
    int speed = 0;  // 速度（编码器单位，具体含义依驱动）
    int load = 0;   // 负载/电流等（若可读）
    bool success = false; // 本次读是否成功
    
    ros::Time last_load_update; // 上次更新 load 的时间
};


// -------------------- 低频读取 load --------------------
void updateLoadLowFreq(map<int, InternalFeedback>& feedbacks, double interval_sec = 0.5) {
    ros::Time now = ros::Time::now();
    for (auto& kv : feedbacks) {
        int id = kv.first;
        InternalFeedback& fb = kv.second;
        if ((now - fb.last_load_update).toSec() >= interval_sec) {
            int load_val = sm_st.ReadLoad(id);
            if (load_val != -1) {
                fb.load = load_val;
                fb.last_load_update = now;
            }
        }
    }
}


// -------------------- 辅助函数 --------------------

// 把弧度转为编码器值（与你原来的实现保持一致）
inline int rad2enc(double rad) {
    return static_cast<int>(rad / 3.141592653589793 * 2048.0 + 2048.0);
}

// 把编码器值转为弧度（用于发布）
inline double enc2rad(int enc) {
    return (enc - 2048) * 3.141592653589793 / 2048.0;
}

// 计算 "带符号的最短误差"：返回范围 (-RES/2, RES/2]
// 说明：如果 target==current，则差为0；如果 target 在 current 的前方或后方会返回最短路径的带符号差
inline int shortest_signed_error(int target, int current) {
    // 先把差映射到 [0, RES)
    int diff = ( (target - current) % SERVO_ENCODER_RES + SERVO_ENCODER_RES ) % SERVO_ENCODER_RES;
    // 把大于半圈的部分转为负向差
    if (diff > SERVO_ENCODER_RES/2) diff -= SERVO_ENCODER_RES;
    return diff;
}

// -------------------- ROS 回调：更新目标位置 --------------------
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 你的 JointState 里关节索引是 11..20 对应舵机 1..10
    if (msg->position.size() < 21) return;
    for (int idx = 11; idx <= 20; ++idx) {
        int servo_id = idx - 10;
        double target_rad = msg->position[idx];
        int target_enc = rad2enc(target_rad);
        target_positions[servo_id] = target_enc;
    }
}

// 初始化阶段调用（只调用一次）
void syncReadInit() {
    const uint8_t RXPACKET_LEN = 4; // 2字节位置 + 2字节速度
    sm_st.syncReadBegin(static_cast<int>(servo_IDs.size()), RXPACKET_LEN);
}

// 结束时调用（只调用一次）
void syncReadClose() {
    sm_st.syncReadEnd();
}

// 每次批量读调用（循环内多次调用）
map<int, InternalFeedback> batch_read_feedback() {
    map<int, InternalFeedback> feedbacks;
    const uint8_t RXPACKET_LEN = 4;
    uint8_t rxPacket[RXPACKET_LEN];

    // 发送同步读指令
    sm_st.syncReadPacketTx(const_cast<uint8_t*>(servo_IDs.data()), servo_IDs.size(),
                           SMS_STS_PRESENT_POSITION_L, RXPACKET_LEN);

    for (size_t i = 0; i < servo_IDs.size(); ++i) {
        uint8_t id = servo_IDs[i];
        InternalFeedback fb;
        if (!sm_st.syncReadPacketRx(id, rxPacket)) {
            ROS_WARN_THROTTLE(5, "Sync read failed for ID %d", id);
        } else {
            fb.pos = sm_st.syncReadRxPacketToWrod(15);
            fb.speed = sm_st.syncReadRxPacketToWrod(15);
            fb.load = 0;
            fb.success = true;
        }
        feedbacks[id] = fb;
    }

    return feedbacks;
}


// -------------------- 批量写位置（基于批量读的结果决定速度） --------------------
void batch_set_positions(const map<int, InternalFeedback>& feedbacks) {
    for (auto id : servo_IDs) {
        auto it_target = target_positions.find(id);
        if (it_target == target_positions.end()) continue; // 没有目标就跳过
        int target_pos = it_target->second;

        // 取得本次读到的当前位置（带 success 标志）
        auto it_fb = feedbacks.find(id);
        if (it_fb == feedbacks.end() || !it_fb->second.success) {
            // 没读到当前位置就跳过发送控制（更稳妥）
            continue;
        }
        int current_pos = it_fb->second.pos;

        // 计算带符号的最短误差（考虑环绕）
        int signed_err = shortest_signed_error(target_pos, current_pos);
        int err_mag = std::abs(signed_err); // 幅值用于速度映射  所以取了绝对值

        // 如果误差小于阈值，停下来（或把速度设为0）
        int speed = 0;
        if (err_mag >= ERROR_THRESHOLD) {
            // 简单线性映射：误差越大速度越大，映射到 [MIN_SPEED, SPEED_MAX]
            // 这里用 err_mag / SERVO_ENCODER_RES 的比例来缩放到 SPEED_MAX
            int mapped = (err_mag * SPEED_MAX) / SERVO_ENCODER_RES;
            mapped = std::max(MIN_SPEED, mapped);
            speed = std::min(SPEED_MAX, mapped);
        } else {
            speed = 0;
        }

        // 写位置命令：WritePosEx(id, position, speed, acceleration)
        // 注意：WritePosEx 是以绝对位置为目标，speed 是运行速度 (无符号)
        sm_st.WritePosEx(id, target_pos, speed, ACCELERATION);
    }
}

// -------------------- 主函数 --------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh("~");  // "~" 私有命名空间，读取节点参数

    std::string port;
    // 从参数服务器读取串口设备路径，默认/dev/ttyUSB0
    nh.param<std::string>("port", port, "/dev/ttyUSB0");

    ROS_INFO("打开串口 %s，波特率 1000000", port.c_str());

    if (!sm_st.begin(1000000, port.c_str())) {
        ROS_ERROR("串口初始化失败");
        return 1;
    }

    // 启动时用当前实际位置初始化 target_positions（避免跳动）
    for (auto id : servo_IDs) {
        int pos = sm_st.ReadPos(id);
        target_positions[id] = pos;
    }

    ros::Subscriber sub = nh.subscribe("/command_joint_states", 10, commandCallback);
    ros::Publisher fb_pub = nh.advertise<sensor_msgs::JointState>("/feedback_feetech_joint_states", 10);

    ros::Rate rate(100);
    ros::Time last_print_time = ros::Time::now();

	syncReadInit();
	
	
    while (ros::ok()) {
        ros::spinOnce();

        // 1) 批量读
        auto feedbacks = batch_read_feedback();
        
                // 2) 基于反馈进行批量写（速度动态调整）
        batch_set_positions(feedbacks);
        
    // 2) 低频更新 load
    updateLoadLowFreq(feedbacks, 1.3); // 每 0.5 秒更新一次



        // 3) 发布反馈话题（JointState）
        sensor_msgs::JointState fb_msg;
        fb_msg.header.stamp = ros::Time::now();
        fb_msg.name.resize(servo_IDs.size());
        fb_msg.position.resize(servo_IDs.size());
        fb_msg.velocity.resize(servo_IDs.size());
        fb_msg.effort.resize(servo_IDs.size());

        for (size_t i = 0; i < servo_IDs.size(); ++i) {
            int id = servo_IDs[i];
            fb_msg.name[i] = "servo_" + std::to_string(id);

            auto it_fb = feedbacks.find(id);
            if (it_fb == feedbacks.end() || !it_fb->second.success) {
                fb_msg.position[i] = 0.0;
                fb_msg.velocity[i] = 0.0;
                fb_msg.effort[i] = 0.0;
                continue;
            }
            const InternalFeedback& fb = it_fb->second;
            fb_msg.position[i] = enc2rad(fb.pos);
            // 注意：这里直接用 enc->rad 把 speed 转为弧度/单位时间 只是近似（取决于 speed 的单位）
            fb_msg.velocity[i] =  static_cast<double>(fb.speed) / 2048.0 * M_PI;
            fb_msg.effort[i] = (static_cast<double>(fb.load) / 1000.0) * servo_max_torque[i];
        }
        fb_pub.publish(fb_msg);

        // 4) 每秒打印一次详细状态（ID, target, current, signed error）
        if ((ros::Time::now() - last_print_time).toSec() > 1.0) {
            std::ostringstream oss;
            oss << "\nServo status (enc):\n";
            oss << "ID\tTarget\tCurrent\tSignedErr\n";
            oss << std::setfill(' ');
            for (auto id : servo_IDs) {
                int target = target_positions.count(id) ? target_positions.at(id) : 0;
                int current = 0;
                int signed_err = 0;
                int load_val = 0;
                auto it_fb = feedbacks.find(id);
                if (it_fb != feedbacks.end() && it_fb->second.success) {
                    current = it_fb->second.pos;
                    signed_err = shortest_signed_error(target, current); // 带符号的最短差
                    load_val = it_fb->second.load;
                }
                oss << std::setw(2) << id << "\t"
                    << std::setw(5) << target << "\t"
                    << std::setw(5) << current << "\t"
                    << std::setw(6) << signed_err << "\t"
                    << std::setw(5) << load_val << "\n";
                    
            }
            ROS_INFO_STREAM(oss.str());
            last_print_time = ros::Time::now();
        }

        rate.sleep();
    }
    syncReadClose();

    sm_st.end();
    return 0;
}

