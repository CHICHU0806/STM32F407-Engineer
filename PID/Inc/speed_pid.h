//
// Created by 20852 on 2025/9/11.
//

#ifndef STARTM3508_PID_H
#define STARTM3508_PID_H
#include <sys/_stdint.h>

#ifdef __cplusplus
class SpeedPID {
public:
    SpeedPID(float kp, float ki, float kd, float max_out, float max_iout);

    void PID_Clear();

    // dt: 采样周期，单位秒
    float PID_Calculate(float target_speed, float actual_speed, float dt);

private:
    float Kp;
    float Ki;
    float Kd;

    float maxOutput;    // 输出限幅
    float maxIntegral;  // 积分限幅

    float integral;
    float prevError;
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