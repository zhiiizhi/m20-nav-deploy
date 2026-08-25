/**
 * @file motion_state_feedback.hpp
 * @brief motion state feedback 
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-11-07
 * 
 * @copyright Copyright (c) 2025  DeepRobotics
 * 
 */
#pragma once
#include <atomic>
#include <iostream>
#include <cstring>
#include "common_types.h"

class MotionStateFeedback
{
private:
    std::atomic<uint8_t> current_state_{0};
    uint8_t current_gait_;
    uint8_t last_state_;
    uint8_t last_gait_;

    Vec3f current_vel_;
    Vec3f cmd_vel_;
    Vec3f max_vel_;

    union{
        struct{
            uint32_t joint_pos_limit : 1;
            uint32_t posture_limit : 1;
            uint32_t reserved : 30;
        }error_code_bit_;
        uint32_t error_code_;
    };
public:
    MotionStateFeedback(/* args */){
        current_state_ = 0;
        current_gait_ = 0;
        last_state_ = 0;
        last_gait_ = 0;
        current_vel_.setZero();
        cmd_vel_.setZero();
        max_vel_.setZero();
        error_code_ = 0;
    }
    ~MotionStateFeedback(){

    }

    void UpdateCurrentState(int state){
        uint8_t s = uint8_t(state);
        uint8_t cur = current_state_.load(std::memory_order_relaxed);
        if(s != cur){
            last_state_ = cur;
            current_state_.store(s, std::memory_order_release);
        }
    }

    void UpdateCurrentGait(uint8_t gait){
        if(gait != current_gait_){
            last_gait_ = current_gait_;
            current_gait_ = gait;
        }
    }

    void ClearMotionError(){error_code_ = 0;}
    void SetJointPosLimitError(){
        error_code_bit_.joint_pos_limit = 1;
    }
    void SetPostureLimitError(){
        error_code_bit_.posture_limit = 1;
    }

    inline uint8_t GetCurrentState(){return current_state_.load(std::memory_order_acquire);}
    inline uint8_t GetCurrentGait(){return current_gait_;}
    inline uint8_t GetLastState(){return last_state_;}
    inline uint8_t GetLastGait(){return last_gait_;}
};
