//
// Created by 20852 on 2025/10/30.
//

#include "../Inc/masterBoardTask.h"

void MasterBoardTask::run() {
    for (;;) {
        osDelay(10);
    }
}

extern "C" {
static MasterBoardTask master_board_task;

    void MasterBoardTask_Init() {
        master_board_task.start((char*)"MasterBoardTask", 512, osPriorityNormal);
    }

}

