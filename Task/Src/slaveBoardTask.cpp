//
// Created by 20852 on 2025/11/15.
//

#include "slaveBoardTask.h"
#include "bsp_can.h"

void SlaveBoardTask::run() {
    for (;;) {

        osDelay(10);
    }
}

extern "C" {
    static SlaveBoardTask slave_board_task;

    void SlaveBoardTask_Init() {
        slave_board_task.start((char*)"SlaveBoardTask", 512, osPriorityNormal);
    }

}