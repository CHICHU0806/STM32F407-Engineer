//
// Created by 20852 on 2025/9/20.
//

#ifndef STARTM3508_MECANUM_H
#define STARTM3508_MECANUM_H

#pragma once
#include <array>

class Mecanum {
public:
    // 构造函数：传入车长 L 和车宽 W
    Mecanum(float length, float width, float radius = 1.0f);

    // 计算四个轮子的速度
    // 输入: Vx 前后速度, Vy 左右速度, ω 角速度(rad/s)
    // 输出: 四个轮速 (前左, 前右, 后左, 后右)
    std::array<float, 4> Mecanum_Calculate(float Vx, float Vy, float omega);

private:
    float L;   // 车长
    float W;   // 车宽
    float r;   // 轮半径
};

#ifdef __cplusplus
extern "C" {
#endif

    void mecanum_calculate(float Vx, float Vy, float omega);

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_MECANUM_H