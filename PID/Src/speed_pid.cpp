//
// Created by 20852 on 2025/9/11.
//

    #include "../Inc/speed_pid.h"
#include "../Debug-Vars/debug_vars.h"
#include <algorithm> // for std::clamp
#include <stdint.h>

// 构造函数
SpeedPID::SpeedPID(float kp, float ki, float kd, float max_out, float max_iout)
    : Kp(kp), Ki(ki), Kd(kd),
      maxOutput(max_out), maxIntegral(max_iout),
      integral(0.0f), prevError(0.0f) {}

// 清零 PID 状态
void SpeedPID::PID_Clear() {
    integral = 0.0f;
    prevError = 0.0f;
}

// 速度环 PID 计算
float SpeedPID::PID_Calculate(float target_speed, float actual_speed, float dt) {
    float error = target_speed - actual_speed;

    // 微分项
    float derivative = (error - prevError) / dt;

    // 先计算 P+D 部分输出
    float output = Kp * error + Kd * derivative;

    // 条件积分：仅当输出未达到限幅时累加积分
    if (output < maxOutput && output > -maxOutput) {
        integral += Ki * error * dt;
        integral = std::clamp(integral, -maxIntegral, maxIntegral);
    }

    // 总输出
    output += integral;
    output = std::clamp(output, -maxOutput, maxOutput);

    prevError = error;


    // 调试变量
    debug_P = Kp * error;
    debug_I = integral;
    debug_D = Kd * derivative;

    return output;
}

// C接口封装
extern "C" {
    static SpeedPID speed_pid(0.5f, 0.08f, 0.005f, 5000.0f, 200.0f);

    void speed_pid_clear() {
        speed_pid.PID_Clear();
    }

    int16_t speed_pid_calculate(float target_speed, float actual_speed, float dt) {
        float output = speed_pid.PID_Calculate(target_speed, actual_speed, dt);
        return static_cast<int16_t>(output);
    }
}