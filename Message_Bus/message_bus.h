//
// Created by 20852 on 2025/10/17.
//

#ifndef STARTM3508_MESSAGE_BUS_H
#define STARTM3508_MESSAGE_BUS_H

#pragma once
#ifdef __cplusplus
#include <functional>
#include <vector>
#include <cstdint>

struct MotorCmd {
    float target_speed;
};

extern MotorCmd motorCmd;

#endif

#endif //STARTM3508_MESSAGE_BUS_H