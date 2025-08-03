#include "servo_controller/servo.h"   // 如果放在include/servo_controller/目录下，且CMake配置了include

#include <iostream>
#include <unistd.h>
#include <cmath>
#include <csignal>
#include <algorithm>

Servo::PID::PID(float p, float i, float d) : Kp(p), Ki(i), Kd(d) {}

//此函数的作用是初始化舵机对象，设置舵机ID、串口和PID参数
//如果串口未初始化，则进行初始化，并设置舵机为轮式模式
//如果串口初始化失败，则抛出异常
//将当前舵机ID添加到活动舵机ID列表中，并设置反馈
//最后输出初始化完成的日志信息
Servo::Servo(int id, const std::string& port, float Kp, float Ki, float Kd)//这个是构造函数 
    : ServoID(id), serial_port(port), speed_pid(Kp, Ki, Kd) ,acc_pid(Kp, Ki, Kd){

    activeServoIDs.push_back(ServoID);// 将当前舵机ID添加到活动舵机ID列表中
    feedback.id = ServoID;// 设置反馈ID为当前舵机ID
    speed_max = 1000;// 设置最大速度
    acceleration = 50;// 设置加速度


    if (!initialized) {
        LOG_INFO("初始化串口...");
        if (!sm_st.begin(1000000, serial_port.c_str())) {// 初始化串口
            LOG_ERROR("串口初始化失败");
            throw std::runtime_error("串口初始化失败");
        }

        std::signal(SIGINT, signalHandler);// 注册信号处理函数，捕捉 Ctrl+C 信号
        initialized = true;// 设置串口已初始化标志
    }

    LOG_INFO("舵机初始化完成。");
    sm_st.WheelMode(ServoID);// 设置舵机为轮式模式 必须要加
}

Servo::~Servo() {
    stopFlag = true;// 设置停止标志为 true，表示舵机将停止工作
    auto it = std::remove(activeServoIDs.begin(), activeServoIDs.end(), ServoID);// 从活动舵机ID列表中移除当前舵机ID
    activeServoIDs.erase(it, activeServoIDs.end());// 删除已停止的舵机ID
    LOG_INFO("舵机对象销毁。");
}

void Servo::setDefaultAngle(int angle) { default_angle = angle; }// 设置默认角度
void Servo::setSpeedMax(int speed) { speed_max = speed; }// 设置最大速度
void Servo::setAcceleration(int acc) { acceleration = acc; }// 设置加速度
void Servo::setDeadzone(int min_val, int max_val) {// 设置死区 待完善
    deadzone_min = min_val;
    deadzone_max = max_val;
}

void Servo::PID_setAngle_control(int target_angle) {// PID 控制舵机角度
    ServoFeedback feedback = get_feedback(ServoID);
    //update_servo_control_speed_pos_loop(target_angle, feedback.pos);
    float target_speed = pid_calculate(speed_pid, target_angle, feedback.pos, speed_max);// 计算目标速度
    sm_st.WriteSpe(ServoID, int(target_speed), acceleration);// 写入速度和加速度到舵机
}


void Servo::PID_twoloop_control(int target_angle,float Kp,float Ki,float Kd){// PID 双环控制舵机角度 还未测试
    ServoFeedback feedback = get_feedback(ServoID);
    acc_pid.Kp=Kp;
    acc_pid.Ki=Ki;
    acc_pid.Kd=Kd;
    float target_speed = pid_calculate(speed_pid, target_angle, feedback.pos, speed_max);// 计算目标速度
    float temp_acceleration = pid_calculate(acc_pid, target_speed, feedback.speed, acceleration_max);// 计算临时加速度
    sm_st.WriteSpe(ServoID, int(target_speed), int(temp_acceleration));// 写入速度和加速度到舵机
}

void Servo::normally_setAngle_control(int target_angle) {// 普通控制舵机角度 会很抖
    sm_st.WritePosEx(ServoID, target_angle, speed_max, acceleration);
}

void Servo::stop_servo(int id) {// 停止舵机
    sm_st.WriteSpe(id, 0, acceleration);
}

void Servo::terminateAllServos() {// 停止所有舵机
    for (int id : activeServoIDs) {
        sm_st.WriteSpe(id, 0, 50);
    }
    stopFlag = true;
}

bool Servo::isStopped() {// 检查是否停止
    return stopFlag;
}

void Servo::closeSerial() {// 关闭串口
    if (initialized) {
        sm_st.end();
        initialized = false;
        LOG_INFO("串口关闭完成。");
    }
}

Servo::ServoFeedback Servo::get_feedback(int id) {// 获取舵机反馈信息
    ServoFeedback fb;
    if (sm_st.FeedBack(id) != -1) {// 检查舵机反馈是否成功
        fb.pos = sm_st.ReadPos(id);
        fb.speed = sm_st.ReadSpeed(id);
        fb.current = sm_st.ReadCurrent(id);
        fb.load = sm_st.ReadLoad(id);
        fb.move = sm_st.ReadMove(id);
        fb.voltage = sm_st.ReadVoltage(id);
        std::cout << id << "," << fb.pos << "," << fb.speed << ","
                  << fb.load << "," << fb.voltage << ","
                  << fb.temper << "," << fb.move << ","
                  << fb.current << std::endl;
        fb.success = true;
    } else {
        std::cerr << "FeedBack error for ID " << id << std::endl;
        fb.success = false;
    }
    return fb;
}

float Servo::pid_calculate(PID& pid, float target, float current, float out_max) {// PID 计算函数
    float error = target - current;// 计算误差
    if (error > max_value / 2) error -= max_value;// 如果误差大于最大值的一半，则进行环绕处理
    else if (error < -max_value / 2) error += max_value;// 如果误差小于负最大值的一半，则进行环绕处理 （过零处理）

    pid.integral += error;// 积分计算
    pid.integral = std::clamp(pid.integral, -pid.integral_limit, pid.integral_limit);// 限制积分值在一定范围内

    float derivative = error - pid.prev_error;// 计算微分
    pid.output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;// PID 输出计算
    pid.prev_error = error;

    pid.out_max = out_max;
    pid.output = std::clamp(pid.output, -out_max, out_max);// 限制输出在最大值范围内

    if (std::abs(error) < 2.5f) {// 如果误差小于2.5，则认为已经到达目标位置
        pid.output = 0.0f;
        pid.integral = 0.0f;
    }

    std::cout << "----------------------\n";// 调试输出 PID 计算过程
    std::cout << "PID Calculation:\n";
    std::cout << "Target    : " << target << "\n"; 
    std::cout << "Error     : " << error << "\n";
    std::cout << "P (Kp*e)  : " << pid.Kp * error << "\n";
    std::cout << "I (Ki*∑e) : " << pid.Ki * pid.integral << "\n";
    std::cout << "D (Kd*Δe) : " << pid.Kd * derivative << "\n";
    std::cout << "Output    : " << pid.output << "\n";

    return pid.output;
}

void Servo::update_servo_control_speed_pos_loop(float target_pos, float current_pos) {// 更新舵机控制速度和位置循环
    float target_speed = pid_calculate(speed_pid, target_pos, current_pos, speed_max);
    sm_st.WriteSpe(ServoID, int(target_speed), acceleration);
}

void Servo::signalHandler(int signum) {// 信号处理函数，捕捉 Ctrl+C 信号
    const char msg[] = "\n[INFO] 捕捉到 Ctrl+C，准备退出...\n";
    write(STDOUT_FILENO, msg, sizeof(msg) - 1);
    stopFlag = true;
}


