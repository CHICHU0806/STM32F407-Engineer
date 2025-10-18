//
// Created by 20852 on 2025/9/11.
//

#include "../Inc/speed_pid.h"

// C接口封装
extern "C" {
    static SpeedPID speed_pid(0.7f, 0.01f, 0.001f, 5000.0f, 200.0f);

    void speed_pid_clear() {
        speed_pid.Clear();
    }

    int16_t speed_pid_calculate(float target_speed, float actual_speed, float dt) {
        float output = speed_pid.Calculate(target_speed, actual_speed, dt);
        return static_cast<int16_t>(output);
    }
}