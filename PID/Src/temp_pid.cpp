//
// Created by 20852 on 2025/11/25.
//

#include "../Inc/temp_pid.h"
void TempPID::setTarget(float t) {
    target_temp = t;
}

float TempPID::calculate(float currentTemp, float dt) {
    float error = target_temp - currentTemp;

    // 微分项
    float derivative = (error - prevError) / dt;

    // PD
    float output = Kp * error + Kd * derivative;

    // -------- 积分控制（只能加热，不能为负） --------
    if (output < maxOutput) {
        integral += Ki * error * dt;
        if (integral < 0) integral = 0;             // 温控不能制冷
        integral = clamp(integral, 0.0f, maxIntegral);
    }

    output += integral;

    // -------- 禁止负输出 --------
    if (output < 0) output = 0;

    // -------- 限幅 --------
    output = clamp(output, 0.0f, maxOutput);

    prevError = error;
    prevTarget = target_temp;

    return output;
}

extern "C" {
    static TempPID temp_pid(1600.0f, 0.2f, 0.0f, 4500.0f, 4400.0f);

    void temp_pid_clear() {
        temp_pid.Clear();
    }

    int16_t temp_pid_calculate(float target_temp, float current_temp, float dt) {
        temp_pid.setTarget(target_temp);
        return temp_pid.calculate(current_temp, dt);
    }
}