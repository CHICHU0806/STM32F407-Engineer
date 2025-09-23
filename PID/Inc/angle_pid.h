//
// Created by 20852 on 2025/9/21.
//

#ifndef STARTM3508_ANGLE_PID_H
#define STARTM3508_ANGLE_PID_H

#pragma once
#include "PidBase.h"
#include <stdint.h>
#include <cmath>

#ifdef __cplusplus
class AnglePID : public PidBase {
public:
    using PidBase::PidBase;

    // 设置目标角度，便于访问私有成员
    void setTarget(int32_t target) {
        target_angle = target;
    }

    // 在原来基类的计算基础上添加跨零处理
    int16_t calculate(int32_t feedback, float dt) {
        int32_t fb = handleZeroCross(target_angle, feedback);
        float output = Calculate(static_cast<float>(target_angle),static_cast<float>(fb), dt);
        return static_cast<int16_t>(output);
    }

private:
    int32_t target_angle = 0;

    // 跨零处理
    int32_t handleZeroCross(int32_t target, int32_t current) {
        int32_t error = target - current;
        if(error > 4096) current += 8192;
        else if(error < -4096) current -= 8192;
        return current;
    }
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void angle_pid_clear();
    int16_t angle_pid_calculate(int32_t target, int32_t feedback); // 获取输出

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_ANGLE_PID_H