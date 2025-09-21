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

    void setTarget(int32_t target) { target_angle = target; }

    int16_t calculate(int32_t feedback, float dt) {
        int32_t fb = handleZeroCross(target_angle, feedback);
        float output = PidBase::Calculate(static_cast<float>(target_angle),
                                         static_cast<float>(fb), dt);
        return static_cast<int16_t>(output);
    }

    void reset() { Clear(); }

private:
    int32_t target_angle = 0;

    // 跨零处理
    int32_t handleZeroCross(int32_t tar, int32_t cur) {
        int32_t diff = tar - cur;
        if(diff > 4096) cur += 8192;
        else if(diff < -4096) cur -= 8192;
        return cur;
    }
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void angle_pid_clear();
    int16_t angle_pid_calculate(int32_t target, int32_t feedback); // 获取输出

    //int16_t angle_pid_calculate(float setpoint, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_ANGLE_PID_H