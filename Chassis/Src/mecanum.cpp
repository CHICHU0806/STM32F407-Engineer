//
// Created by 20852 on 2025/9/20.
//

#include "../Inc/mecanum.h"

Mecanum::Mecanum(float length, float width, float radius)
    : L(length), W(width), r(radius) {}

// 计算麦轮轮速
std::array<float, 4> Mecanum::Mecanum_Calculate(float Vx, float Vy, float omega) {
    std::array<float, 4> MotorSpeeds; // 前左, 前右, 后右, 后左

    float R = L + W;  // 等效半径

    // 四轮速度公式 (X 型布局)
    MotorSpeeds[0] = ( Vx - Vy - R * omega ) / r;  // 前左
    MotorSpeeds[1] = -( Vx + Vy + R * omega ) / r;  // 前右
    MotorSpeeds[2] = ( Vx + Vy - R * omega ) / r;  // 后左
    MotorSpeeds[3] = -( Vx - Vy + R * omega ) / r;  // 后右

    return MotorSpeeds;
}

extern "C"{
    static Mecanum mecanum(0.6f, 0.42f, 0.08f);

    void mecanum_calculate(float Vx, float Vy, float omega) {
        mecanum.Mecanum_Calculate(Vx, Vy, omega);
    }
}
