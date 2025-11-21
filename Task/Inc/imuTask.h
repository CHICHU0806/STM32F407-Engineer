//
// Created by 20852 on 2025/11/21.
//

#ifndef STARTM3508_IMUTASK_H
#define STARTM3508_IMUTASK_H

#pragma once
#include "../TaskBase.h"
#include "main.h"

#ifdef __cplusplus
class ImuTask : public TaskBase {
public:
    void run() override;   // 继承并实现 run()
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    void ImuTask_Init();

#ifdef __cplusplus
}
#endif



#endif //STARTM3508_IMUTASK_H