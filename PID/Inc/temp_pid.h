//
// Created by 20852 on 2025/11/25.
//

#ifndef STARTM3508_TEMP_PID_H
#define STARTM3508_TEMP_PID_H

#include "PidBase.h"

#ifdef __cplusplus
class TempPID : public PidBase {
public:
    using PidBase::PidBase;

    void setTarget(float t);

    float calculate(float currentTemp, float dt);

private:
    float target_temp = 45.0f;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void temp_pid_clear();
    int16_t temp_pid_calculate(float target_temp, float current_temp, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_TEMP_PID_H