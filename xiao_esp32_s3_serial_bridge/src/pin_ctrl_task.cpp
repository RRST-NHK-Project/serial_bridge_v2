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
#if defined(MODE_IR)
#include "ir/IRremote.hpp"
#endif
#include <Arduino.h>

constexpr uint32_t CTRL_PERIOD_MS = 5; // ピン更新周期（ミリ秒）

void SDM15_Task(void *);
void IR_Task(void *);

HardwareSerial SerialSDM(1);
SDM15 sdm15(SerialSDM);

#if defined(MODE_IR)
namespace {
    constexpr uint32_t IR_DEDUP_WINDOW_MS = 100;
}
#endif

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

void IR_Task(void *pvParameters) {
#if defined(MODE_IR)
    (void)pvParameters;

    IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

    uint64_t last_raw = 0;
    uint32_t last_rx_ms = 0;

    while (1) {
        if (IrReceiver.decode()) {
            const IRData &data = IrReceiver.decodedIRData;
            const uint64_t raw = data.decodedRawData;
            const uint32_t now_ms = millis();
            const bool protocol_allowed =
                (IR_FILTER_ENABLE == 0) || (data.protocol == IR_FILTER_PROTOCOL);

            // 同一信号の連続受信を抑制して、PC側の処理負荷を下げる
            const bool is_duplicate =
                (raw == last_raw) && ((now_ms - last_rx_ms) < IR_DEDUP_WINDOW_MS);

            if (protocol_allowed && !is_duplicate) {
                Tx_16Data[0] = (int16_t)data.protocol;
                Tx_16Data[1] = (int16_t)data.address;
                Tx_16Data[2] = (int16_t)data.command;
                Tx_16Data[3] = (int16_t)data.flags;
                Tx_16Data[4] = (int16_t)data.numberOfBits;
                Tx_16Data[5] = (int16_t)(raw & 0xFFFF);
                Tx_16Data[6] = (int16_t)((raw >> 16) & 0xFFFF);
                Tx_16Data[7] = (int16_t)((raw >> 32) & 0xFFFF);
                Tx_16Data[8] = (int16_t)((raw >> 48) & 0xFFFF);

                last_raw = raw;
                last_rx_ms = now_ms;
            }

            IrReceiver.resume();
        }

        vTaskDelay(pdMS_TO_TICKS(IR_TASK_PERIOD_MS));
    }
#else
    (void)pvParameters;
    vTaskDelete(NULL);
#endif
}
