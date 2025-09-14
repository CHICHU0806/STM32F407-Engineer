//
// Created by 20852 on 2025/9/12.
//

#ifndef STARTM3508_DEBUG_VARS_H
#define STARTM3508_DEBUG_VARS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//基本框架
//extern volatile 类型 变量名；
extern volatile float debug_P;
extern volatile float debug_I;
extern volatile float debug_D;
extern volatile float debug_actual_speed;
extern volatile int16_t debug_ch0;

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_DEBUG_VARS_H