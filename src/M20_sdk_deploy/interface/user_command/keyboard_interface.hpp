// keyboard_interface.hpp
#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <cctype>
#include <mutex>
#include <vector>

// ROS2 for cmd_vel subscription (Nav2 navigation)
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    std::atomic<bool> running_{false};
    std::thread kb_thread_;
    mutable std::mutex keys_mutex_;

    float max_forward_ = 0.7f;
    float max_side_    = 0.5f;
    float max_yaw_     = 0.7f;

    // 模型左右不对称补偿
    float yaw_left_scale_  = 1.0f;   // q 左转：0.7×0.7=0.49
    float yaw_right_scale_ = 1.0f;   // e 右转：0.7×1.8=1.26

    std::unordered_set<char> held_keys_;
    std::unordered_map<char, double> last_seen_time_;
    
    const std::unordered_set<char> velocity_keys_ = {'w', 's', 'a', 'd', 'q', 'e'};
    const double key_timeout_ms_ = 500.0;

    // Nav2 cmd_vel subscriber
    std::atomic<bool> nav_active_{false};
    float nav_fwd_{0.0f}, nav_side_{0.0f}, nav_yaw_{0.0f};
    rclcpp::Node::SharedPtr nav_node_{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_{nullptr};
    std::thread nav_spin_thread_;
    std::atomic<bool> nav_running_{false};
    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    static constexpr double CMD_VEL_TIMEOUT_SEC = 0.5;  // revert to keyboard after timeout

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        nav_fwd_ = static_cast<float>(msg->linear.x);
        nav_side_ = static_cast<float>(msg->linear.y);
        nav_yaw_ = static_cast<float>(msg->angular.z);
        nav_active_ = true;
        last_cmd_vel_time_ = std::chrono::steady_clock::now();
    }

    void nav_spin_loop() {
        while (nav_running_) {
            rclcpp::spin_some(nav_node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Check timeout: if no cmd_vel received, revert to keyboard
            if (nav_active_) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - last_cmd_vel_time_).count();
                if (elapsed > CMD_VEL_TIMEOUT_SEC) {
                    nav_active_ = false;
                    nav_fwd_ = 0.0f;
                    nav_side_ = 0.0f;
                    nav_yaw_ = 0.0f;
                }
            }
        }
    }

    void init_cmd_vel_subscriber() {
        try {
            nav_node_ = rclcpp::Node::make_shared("cmd_vel_bridge");
            cmd_vel_sub_ = nav_node_->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10,
                std::bind(&KeyboardInterface::cmd_vel_callback, this, std::placeholders::_1));
            nav_running_ = true;
            nav_spin_thread_ = std::thread(&KeyboardInterface::nav_spin_loop, this);
            std::cout << "[NAV] cmd_vel subscriber active on /cmd_vel\n";
        } catch (const std::exception& e) {
            std::cerr << "[NAV] Failed to create cmd_vel subscriber: " << e.what() << "\n";
        }
    }

    void ClipNumber(float& num, float low, float high)
    {
        if (num < low) num = low;
        if (num > high) num = high;
    }

    double GetCurrentTimeStamp()
    {
        static auto start = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start).count();
    }

    static void setup_raw_mode()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        termios raw = t;
        raw.c_lflag &= ~(ECHO | ICANON);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    static void restore_terminal()
    {
        termios t{};
        tcgetattr(STDIN_FILENO, &t);
        t.c_lflag |= (ECHO | ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &t);
    }

    void compute_velocity_from_held_keys(float& fwd, float& side, float& yaw)
    {
        fwd = 0.0f;
        side = 0.0f;
        yaw = 0.0f;

        std::lock_guard<std::mutex> lock(keys_mutex_);
        
        if (held_keys_.count('w')) fwd += max_forward_;
        if (held_keys_.count('s')) fwd -= max_forward_;
        if (held_keys_.count('a')) side += max_side_;
        if (held_keys_.count('d')) side -= max_side_;
        if (held_keys_.count('q')) yaw += max_yaw_ * yaw_left_scale_;
        if (held_keys_.count('e')) yaw -= max_yaw_ * yaw_right_scale_;
        
        ClipNumber(fwd, -max_forward_, max_forward_);
        ClipNumber(side, -max_side_, max_side_);
        ClipNumber(yaw, -max_yaw_, max_yaw_);

    }

    void process_mode_command(char k)
    {
        if (k == 'r') {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
            std::cout << "[MODE] Joint Damping\n";
        }
        else if (k == 'z' && (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand
            || msfb_->GetCurrentState() == RobotMotionState::LieDown)) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
            std::cout << "[MODE] Standing Up\n";
        }
        else if (k == 'c' && msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
            std::cout << "[MODE] RL Control\n";
        }
        else if (k == 'x' && (msfb_->GetCurrentState() == RobotMotionState::StandingUp 
            || msfb_->GetCurrentState() == RobotMotionState::RLControlMode)) {
            usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
            std::cout << "[MODE] Lie Down\n";
        }
    }

    void keyboard_loop()
    {
        setup_raw_mode();

        std::cout << "\n╔════════════════════════════════════════════════╗\n"
                  << "║      KEYBOARD TELEOP - MULTI-KEY READY         ║\n"
                  << "╚════════════════════════════════════════════════╝\n"
                  << "  Movement:  W/S (forward/back)  A/D (left/right)\n"
                  << "  Rotation:  Q (CCW)  E (CW)\n"
                  << "  Mode:      R (damping)  Z (stand)  C (control)\n"
                  << "\n";

        char ch;

        while (running_) {
            double now = GetCurrentTimeStamp();
            usr_cmd_->time_stamp = now;

            // Read all available keyboard input
            while (read(STDIN_FILENO, &ch, 1) == 1) {
                char k = std::tolower(static_cast<unsigned char>(ch));

                // Handle mode commands
                if (k == 'r' || k == 'z' || k == 'c' || k == 'x') {
                    process_mode_command(k);
                    continue;
                }

                // Track velocity keys
                if (velocity_keys_.count(k)) {
                    std::lock_guard<std::mutex> lock(keys_mutex_);
                    held_keys_.insert(k);
                    last_seen_time_[k] = now;
                }
            }

            // Remove keys that haven't been seen recently (released)
            {
                std::lock_guard<std::mutex> lock(keys_mutex_);
                std::vector<char> to_remove;
                
                for (char k : held_keys_) {
                    if (now - last_seen_time_[k] > key_timeout_ms_) {
                        to_remove.push_back(k);
                    }
                }
                
                for (char k : to_remove) {
                    held_keys_.erase(k);
                    last_seen_time_.erase(k);
                }
            }

            // Compute velocity from all currently held keys
            float fwd = 0.0f, side = 0.0f, yaw = 0.0f;

            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                compute_velocity_from_held_keys(fwd, side, yaw);
            }

            // Nav2 takes priority when active
            float final_fwd = nav_active_ ? nav_fwd_ : fwd;
            float final_side = nav_active_ ? nav_side_ : side;
            float final_yaw = nav_active_ ? nav_yaw_ : yaw;

            usr_cmd_->forward_vel_scale  = final_fwd;
            usr_cmd_->side_vel_scale     = final_side;
            usr_cmd_->turnning_vel_scale = final_yaw;

            if (msfb_->GetCurrentState() == RobotMotionState::RLControlMode && nav_active_) {
                std::cout << "\r[NAV] vel: " << final_fwd << "  " << final_side << "  " << final_yaw << std::flush;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        restore_terminal();
        std::cout << "\n[KEYBOARD] Stopped.\n";
    }

public:
    KeyboardInterface(RobotName robot_name) : UserCommandInterface(robot_name)
    {
        std::cout << "[KeyboardInterface] Initialized with multi-key support\n";
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }

    ~KeyboardInterface() 
    { 
        Stop(); 
    }

    void Start() override
    {
        if (running_) return;
        init_cmd_vel_subscriber();
        running_ = true;
        kb_thread_ = std::thread(&KeyboardInterface::keyboard_loop, this);
    }

    void Stop() override
    {
        running_ = false;
        if (kb_thread_.joinable()) {
            kb_thread_.join();
        }

        nav_running_ = false;
        if (nav_spin_thread_.joinable()) {
            nav_spin_thread_.join();
        }
        if (nav_node_) {
            nav_node_.reset();
        }

        std::lock_guard<std::mutex> lock(keys_mutex_);
        held_keys_.clear();
        last_seen_time_.clear();

        usr_cmd_->forward_vel_scale = 0.0f;
        usr_cmd_->side_vel_scale = 0.0f;
        usr_cmd_->turnning_vel_scale = 0.0f;
    }

    UserCommand* GetUserCommand() override 
    { 
        return usr_cmd_; 
    }

    void set_max_velocities(float fwd, float side, float yaw)
    {
        max_forward_ = std::abs(fwd);
        max_side_    = std::abs(side);
        max_yaw_     = std::abs(yaw);
        std::cout << "[CONFIG] Max velocities: fwd=" << max_forward_ 
                  << " side=" << max_side_ 
                  << " yaw=" << max_yaw_ << "\n";
    }
};