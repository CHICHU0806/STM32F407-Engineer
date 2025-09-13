//
// Created by 20852 on 2025/9/12.
//

//此文件用于定义一些调试变量，方便在调试器中观察
#include "debug_vars.h"

// ==== 定义调试变量 ====
//基本框架
//volatile 类型 变量名:
volatile float debug_P = 0.0f;
volatile float debug_I = 0.0f;
volatile float debug_D = 0.0f;
volatile float debug_actual_speed = -1.0f;