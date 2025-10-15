//
// Created by 20852 on 2025/10/14.
//

#ifndef STARTM3508_UARTTASK_H
#define STARTM3508_UARTTASK_H


#pragma once
#include "TaskBase.h"

#ifdef __cplusplus
class UARTTask : public TaskBase{
public:
    void run() override;
};
#endif

#ifdef __cplusplus
extern "C" {
#endif

    void UartTask_Init();

#ifdef __cplusplus
}
#endif


#endif //STARTM3508_UARTTASK_H