//
// Created by 20852 on 2025/11/15.
//

#ifndef STARTM3508_SLAVEBOARDTASK_H
#define STARTM3508_SLAVEBOARDTASK_H


#pragma once
#include "../TaskBase.h"
#include "main.h"

#ifdef __cplusplus
class SlaveBoardTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void SlaveBoardTask_Init();

#ifdef __cplusplus
}
#endif




#endif //STARTM3508_SLAVEBOARDTASK_H