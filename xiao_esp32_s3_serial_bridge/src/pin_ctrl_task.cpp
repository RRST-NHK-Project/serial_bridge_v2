/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void SDM15_Task(void *);
void SDM15_Input();

// ================= TASK =================

void SDM15_Task(void *pvParameters) {
    (void)pvParameters; // 未使用パラメータの警告回避

    SDM15_init(); // 初期化関数呼び出し

    while (true) {
        SDM15_Input(); // 入力処理関数呼び出し

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS)); // タスクの周期を制御
    }
}

void SDM15_Input() {
    ;
}