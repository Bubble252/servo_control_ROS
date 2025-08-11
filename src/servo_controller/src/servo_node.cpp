#include "servo_controller/ServoFeedback.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "servo_controller/servo.h"
#include <map>
#include <string>

// Max torque for each servo in Nm (adjust as needed)
double servo_max_torque[10] = {10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3,10.3};

std::map<int, Servo*> servo_map;
std::map<int, int> target_positions;  // servo_id -> target encoder value

inline int rad2enc(double rad) {
    return static_cast<int>(rad / 3.14 * 2048.0 + 2048.0);
}

inline double enc2rad(int enc) {
    return (enc - 2048) * 3.14 / 2048.0;
}

int jointIndexToServoId(int idx) {
    return idx - 10; // 11->1 ...
}

void commandCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ROS_INFO("Received control command, position size=%lu", msg->position.size());

    if (msg->position.size() < 21) {
        ROS_WARN("JointState.position length less than 21");
        return;
    }

    for (int idx = 11; idx <= 20; ++idx) {
        int servo_id = jointIndexToServoId(idx);
        double target_rad = msg->position[idx];
        int target_enc = rad2enc(target_rad);

        // Just update the target position in the map
        target_positions[servo_id] = target_enc;

        ROS_DEBUG("Updated target for servo ID=%d: angle(rad)=%.3f, encoder=%d", servo_id, target_rad, target_enc);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_controller_node");
    ros::NodeHandle nh;

    ROS_INFO("servo_controller_node started");

    // Initialize all servos once
    for (int id = 1; id <= 10; ++id) {
        try {
            servo_map[id] = new Servo(id, "/dev/ttyUSB0", 0.1f, 0.01f, 0.05f);
            servo_map[id]->setSpeedMax(1000);
            servo_map[id]->setAcceleration(50);
            // Initialize target_positions with current pos to avoid jump
            auto fb = servo_map[id]->get_feedback(id);
            target_positions[id] = fb.pos;
            ROS_INFO("Initialized servo ID=%d on /dev/ttyUSB0", id);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize servo ID=%d: %s", id, e.what());
        }
    }

    ros::Subscriber sub = nh.subscribe("/command_joint_states", 10, commandCallback);
    ros::Publisher fb_pub = nh.advertise<sensor_msgs::JointState>("/feedback_feetech_joint_states", 10);

    ros::Rate rate(50); // 50Hz

    while (ros::ok()) {
        ros::spinOnce();

        // Send commands to all servos based on stored targets
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


        // Publish feedback
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


        fb_pub.publish(fb_msg);

        rate.sleep();
    }

    ROS_INFO("Exiting and cleaning up");
    for (auto& pair : servo_map) {
        delete pair.second;
    }
    servo_map.clear();

    return 0;
}

