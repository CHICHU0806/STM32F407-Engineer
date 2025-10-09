//
// Created by 20852 on 2025/9/21.
//

#include "angle_pid.h"

void AnglePID::setTarget(int32_t target) {
    target_angle = target;
}

int16_t AnglePID::calculate(int32_t feedback, float dt) {
    int32_t fb = handleZeroCross(target_angle, feedback);
    float output = Calculate(static_cast<float>(target_angle), static_cast<float>(fb), dt);
    return static_cast<int16_t>(output);
}

int32_t AnglePID::handleZeroCross(int32_t target, int32_t current) {
    int32_t error = target - current;
    if (error > 4096) {
        current += 8192;
    } else if (error < -4096) {
        current -= 8192;
    }
    return current;
}

// C接口封装
extern "C" {
    //static AnglePID angle_pid(2.0f, 0.0f, 0.1f, 5000.0f, 5000.0f);
    static AnglePID angle_pid(10.0f, 0.8f, 0.08f, 5000.0f, 5000.0f);

    void angle_pid_clear() {
        angle_pid.Clear();
    }

    int16_t angle_pid_calculate(int32_t target, int32_t actual_angle, float dt) {
        angle_pid.setTarget(target);
        return angle_pid.calculate(actual_angle,dt);
    }
}