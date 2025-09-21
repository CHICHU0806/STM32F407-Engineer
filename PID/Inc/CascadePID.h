//
// Created by 20852 on 2025/9/21.
//

#ifndef STARTM3508_CASCADEPID_H
#define STARTM3508_CASCADEPID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    // 串级 PID 的 C 接口
    int16_t cascade_pid_calculate(float target_angle, float actual_angle, float actual_speed, float dt);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_CASCADEPID_H