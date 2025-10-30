//
// Created by 20852 on 2025/10/30.
//

#ifndef STARTM3508_MASTERBOARDTASK_H
#define STARTM3508_MASTERBOARDTASK_H

#pragma once
#include "../TaskBase.h"
#include "main.h"

#ifdef __cplusplus
class MasterBoardTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void MasterBoardTask_Init();

#ifdef __cplusplus
}
#endif




#endif //STARTM3508_MASTERBOARDTASK_H