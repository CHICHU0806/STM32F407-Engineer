//
// Created by 20852 on 2025/9/19.
//

#ifndef STARTM3508_MOTORTASK_H
#define STARTM3508_MOTORTASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class MotorTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void MotorTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_MOTORTASK_H