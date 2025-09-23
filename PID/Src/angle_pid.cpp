//
// Created by 20852 on 2025/9/21.
//

#include "angle_pid.h"


// C接口封装
extern "C" {
    static AnglePID angle_pid(2.0f, 0.0f, 0.1f, 5000.0f, 5000.0f);

    void angle_pid_clear() {
        angle_pid.Clear();
    }

    int16_t angle_pid_calculate(int32_t target, int32_t feedback) {
        angle_pid.setTarget(target);
        return angle_pid.calculate(feedback,0.01f);
    }

    // int16_t angle_pid_calculate(float target_angle, float actual_angle, float dt) {
    //     float output = angle_pid.Calculate(target_angle, actual_angle, dt);
    //     return static_cast<int16_t>(output);
    // }
}