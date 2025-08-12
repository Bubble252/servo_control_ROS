#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <string>
#include <vector>
#include <atomic>
#include <map>

#define LOG_INFO(msg) std::cerr << "[INFO] " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

#include "SCServo.h"

class Servo {
public:
    Servo(int id, const std::string& port, float Kp = 1.0f, float Ki = 0.0f, float Kd = 0.01f);
    ~Servo();

    void setDefaultAngle(int angle);
    void setSpeedMax(int speed);
    void setAcceleration(int acc);
    void setDeadzone(int min_val, int max_val);

    void PID_setAngle_control(int target_angle);
    void PID_twoloop_control(int target_angle, float Kp, float Ki, float Kd);


    void normally_setAngle_control(int target_angle);
    void stop_servo(int id);

    static void terminateAllServos();
    static bool isStopped();
    static void closeSerial();
    
    
    
    struct ServoFeedback {
        int id = 1;
        int pos = -1;
        int speed = -1;
        int current = -1;
        int load = -1;
        int move = -1;
        int voltage = -1;
        int temper = -1;
        bool success = false;
    };
    ServoFeedback get_feedback(int id);

private:
    struct PID {
        float Kp, Ki, Kd;
        float integral = 0.0f;
        float prev_error = 0.0f;
        float output = 0.0f;
        float out_max = 0.0f;
        float integral_limit = 1000.0f;

        PID(float p, float i, float d);
    };



    
    float pid_calculate(PID& pid, float target, float current, float out_max);
    void update_servo_control_speed_pos_loop(float target_pos, float current_pos);

    static void signalHandler(int signum);

    int ServoID;
    std::string serial_port;
    int default_angle = 512;
    int speed_max;
    int acceleration;
    int max_value = 4096;
    int deadzone_min = -1;
    int deadzone_max = -1;
    int acceleration_max = 1000;

    PID speed_pid;
    PID acc_pid;
    ServoFeedback feedback;

    static inline bool initialized = false;
    static inline std::atomic<bool> stopFlag = false;
    static inline std::vector<int> activeServoIDs;
    static inline SMS_STS sm_st;
};

#endif // SERVO_CONTROLLER_H


