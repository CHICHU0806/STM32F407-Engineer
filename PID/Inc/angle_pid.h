//
// Created by 20852 on 2025/9/21.
//

#ifndef STARTM3508_ANGLE_PID_H
#define STARTM3508_ANGLE_PID_H

#pragma once
#include "PidBase.h"

#ifdef __cplusplus
class AnglePID : public PidBase {
public:
    using PidBase::PidBase;

    // 设置目标角度，便于访问私有成员
    void setTarget(int32_t target) ;

    // 在原来基类的计算基础上添加跨零处理
    int16_t calculate(int32_t feedback, float dt) ;
private:
    int32_t target_angle = 0;

    // 跨零处理
    int32_t handleZeroCross(int32_t target, int32_t current) ;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void angle_pid_clear();
    int16_t angle_pid_calculate(int32_t target, int32_t actual_angle, float dt); // 获取输出

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_ANGLE_PID_H