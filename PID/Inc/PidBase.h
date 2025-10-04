//
// Created by 20852 on 2025/9/21.
//

#ifndef STARTM3508_PIDBASE_H
#define STARTM3508_PIDBASE_H

#pragma once
#include "magictools.h"
#include "stdint.h"

#ifdef __cplusplus

class PidBase {
public:
    PidBase(float kp, float ki, float kd, float max_out, float max_iout)
    : Kp(kp), Ki(ki), Kd(kd),
      maxOutput(max_out), maxIntegral(max_iout),
      integral(0.0f), prevError(0.0f) {}

    virtual ~PidBase() = default; // 虚析构，保证派生类能正确析构

    // 清零
    virtual void Clear() {
        integral = 0.0f;
        prevError = 0.0f;
    }

    // PID 计算函数
    virtual float Calculate(float target, float actual, float dt) {
        float error = target - actual;

        // 检测是否切换了目标方向（正↔负）
        if ((prevTarget * target) < 0.0f) {
            integral = 0.0f; // 清除历史积分，防止反向时积分抵消不对称
        }

        float derivative = (error - prevError) / dt;
        float output = Kp * error + Kd * derivative;

        // 积分防饱和
        if (output < maxOutput && output > -maxOutput) {
            integral += Ki * error * dt;
            integral = clamp(integral, -maxIntegral, maxIntegral);
        }

        output += integral;
        output = clamp(output, -maxOutput, maxOutput);

        prevError = error;
        prevTarget = target;
        return output;
    }

protected:
    float Kp, Ki, Kd;       // PID 参数
    float maxOutput;        // 输出限幅
    float maxIntegral;      // 积分限幅
    float integral;         // 积分项
    float prevError;        // 上一次误差
    float prevTarget = 0.0f; // 上一次目标值
};
#endif

#endif //STARTM3508_PIDBASE_H