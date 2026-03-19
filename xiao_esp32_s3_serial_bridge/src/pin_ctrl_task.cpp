/*====================================================================
<pin_ctrl_task.cpp>
・ピン操作関連の関数とタスクの実装ファイル
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "SDM15.h"
#include "defs.hpp"
#include "driver/pcnt.h"
#include "frame_data.hpp"
#include "pin_ctrl_init.hpp"
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void SDM15_Task(void *);

HardwareSerial SerialSDM(1);
SDM15 sdm15(SerialSDM);

// ================= TASK =================

void SDM15_Task(void *pvParameters) {

    SerialSDM.begin(460800, SERIAL_8N1, SDM15_TX, SDM15_RX);

    // 高速通信対策
    SerialSDM.setRxBufferSize(2048);

    vTaskDelay(pdMS_TO_TICKS(1000)); // 安定待ち

    sdm15.StartScan();

    while (1) {

        if (SerialSDM.available()) {

            ScanData data = sdm15.GetScanData();

            if (!data.checksum_error) {

                // 受信データを送信用配列に格納
                Tx_16Data[0] = int16_t(data.distance);
                Tx_16Data[1] = int16_t(data.intensity);
                Tx_16Data[2] = int16_t(data.disturb);

            } else {
                ; // エラー処理など
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CTRL_PERIOD_MS));
    }
}
