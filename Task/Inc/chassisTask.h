//
// Created by 20852 on 2025/9/20.
//

#ifndef STARTM3508_CHASSISTASK_H
#define STARTM3508_CHASSISTASK_H

#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class ChassisTask : public TaskBase{
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void ChassisTask_Init();

#ifdef __cplusplus
}
#endif

#endif //STARTM3508_CHASSISTASK_H