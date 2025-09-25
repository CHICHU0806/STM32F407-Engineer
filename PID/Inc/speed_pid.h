//
// Created by 20852 on 2025/9/11.
//

#ifndef STARTM3508_PID_H
#define STARTM3508_PID_H

#pragma once
#include "PidBase.h"

#ifdef __cplusplus
class SpeedPID : public PidBase {
public:
    using PidBase::PidBase;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void speed_pid_clear();
    int16_t  speed_pid_calculate(float setpoint, float feedback, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_PID_H