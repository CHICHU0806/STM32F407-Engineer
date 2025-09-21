//
// Created by 20852 on 2025/9/21.
//
#include "speed_pid.h"
#include "angle_pid.h"
#include "CascadePID.h"

// extern "C" {
//     void cascade_pid_clear() {
//         speed_pid_clear();
//         angle_pid_clear();
//     }
//
//     static AnglePID angle_pid(5.0f, 0.0f, 0.1f, 5000.0f, 50.0f);
//     static SpeedPID speed_pid(1.0f, 0.0f, 0.0f, 2000.0f, 500.0f);
//
//     int16_t cascade_pid_calculate(float target_angle, float actual_angle,
//                                   float actual_speed, float dt) {
//         // 外环：角度 → 目标速度
//         float target_speed = angle_pid_calculate(target_angle, actual_angle, dt);
//
//         // 给 target_speed 限幅，保证足够克服静摩擦
//         const float min_speed_threshold = 1.0f; // 可根据实际微调
//         if (fabs(target_speed) < min_speed_threshold) {
//             target_speed = (target_speed >= 0) ? min_speed_threshold : -min_speed_threshold;
//         }
//
//         // 内环：速度 → 电流输出
//         int16_t output = speed_pid_calculate(target_speed, actual_speed, dt);
//
//         // 输出方向修正（确保正负与误差一致）
//         if ((target_angle - actual_angle) < 0) output = -abs(output);
//         else output = abs(output);
//
//         return output;
//     }
// }
