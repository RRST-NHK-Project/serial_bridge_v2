/*====================================================================
Project: esp32_serial_bridge_runtime
Target board: ESP32 Dev Module

Description:
  ROS 2・マイコン間の通信を行うserial_bridgeパッケージのマイコン側プログラム。(runtime版)
  runtime版はROS 2からのモード指定が可能です。IDをconfig.hppで設定するだけで使用可能です。
  このファイル(main.cpp)を直接編集しないこと。
  詳細はREADME.mdを参照してください。

Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "config.hpp"
#include "defs.hpp"
#include "pid_task.hpp"
#include "pin_ctrl_task.hpp"
#include "robomas.hpp"
#include "runtime_mode.hpp"
#include "serial_task.hpp"
#include <Arduino.h>

#if (defined(MODE_RUNTIME) + defined(MODE_OUTPUT) + defined(MODE_INPUT) + defined(MODE_IO) + \
     defined(MODE_ROBOMAS) + defined(MODE_ROBOMAS_PLUS_OUTPUT) + defined(MODE_ROBOMAS_PLUS_INPUT) + defined(MODE_ROBOMAS_PLUS_IO) + defined(MODE_DEBUG)) != 1
#error "Invalid mode configuration. Please define exactly one mode in config.hpp."
#endif

#if defined(MODE_RUNTIME)
namespace {

    TaskHandle_t output_task_handle = NULL;
    TaskHandle_t input_task_handle = NULL;
    TaskHandle_t io_task_handle = NULL;
    TaskHandle_t robomas_io_task_handle = NULL;
    TaskHandle_t m3508_task_handle = NULL;
    TaskHandle_t pid_task_handle = NULL;

    bool robomas_initialized = false;

    void stop_mode_tasks() {
        if (output_task_handle != NULL) {
            vTaskDelete(output_task_handle);
            output_task_handle = NULL;
        }
        if (input_task_handle != NULL) {
            vTaskDelete(input_task_handle);
            input_task_handle = NULL;
        }
        if (io_task_handle != NULL) {
            vTaskDelete(io_task_handle);
            io_task_handle = NULL;
        }
        if (robomas_io_task_handle != NULL) {
            vTaskDelete(robomas_io_task_handle);
            robomas_io_task_handle = NULL;
        }
        if (m3508_task_handle != NULL) {
            vTaskDelete(m3508_task_handle);
            m3508_task_handle = NULL;
        }
        if (pid_task_handle != NULL) {
            vTaskDelete(pid_task_handle);
            pid_task_handle = NULL;
        }
    }

    void ensure_robomas_init() {
        if (!robomas_initialized) {
            robomas_init();
            robomas_initialized = true;
        }
    }

    void start_mode_tasks(RuntimeMode mode) {
        switch (mode) {
        case RT_MODE_OUTPUT:
            xTaskCreate(Output_Task, "Output_Task", 2048, NULL, 11, &output_task_handle);
            break;

        case RT_MODE_INPUT:
            xTaskCreate(Input_Task, "Input_Task", 1024, NULL, 4, &input_task_handle);
            break;

        case RT_MODE_IO:
            xTaskCreate(IO_Task, "IO_Task", 2048, NULL, 11, &io_task_handle);
            break;

        case RT_MODE_ROBOMAS:
            ensure_robomas_init();
            xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, &m3508_task_handle);
            xTaskCreate(PID_Task, "PID_Task", 2048, NULL, 11, &pid_task_handle);
            break;

        case RT_MODE_ROBOMAS_PLUS_OUTPUT:
            ensure_robomas_init();
            xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, &m3508_task_handle);
            xTaskCreate(Output_Task, "Output_Task", 2048, NULL, 8, &output_task_handle);
            break;

        case RT_MODE_ROBOMAS_PLUS_INPUT:
            ensure_robomas_init();
            xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, &m3508_task_handle);
            xTaskCreate(Input_Task, "Input_Task", 1024, NULL, 4, &input_task_handle);
            break;

        case RT_MODE_ROBOMAS_PLUS_IO:
            ensure_robomas_init();
            xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, &m3508_task_handle);
            xTaskCreate(ROBOMAS_IO_Task, "ROBOMAS_IO_Task", 2048, NULL, 11, &robomas_io_task_handle);
            break;

        case RT_MODE_DEBUG:
            xTaskCreate(PID_Task, "PID_Task", 2048, NULL, 11, &pid_task_handle);
            break;

        case RT_MODE_NONE:
        default:
            break;
        }
    }

    void modeManagerTask(void *) {
        RuntimeMode current_mode = RT_MODE_NONE;

        while (1) {
            const RuntimeMode next_mode = get_runtime_mode();

            if (next_mode != current_mode) {
                stop_mode_tasks();
                start_mode_tasks(next_mode);
                current_mode = next_mode;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

} // namespace
#endif
// ================= SETUP =================

void setup() {

    // ボーレートは実機テストしながら調整する予定
    Serial.begin(115200);

    delay(200);
    delay(100 * DEVICE_ID); // 安定待ち, IDごとに開始タイミングをずらす

    pinMode(LED, OUTPUT);

    // ready
    for (int i = 0; i < DEVICE_ID; i++) {
        digitalWrite(LED, HIGH);
        delay(50);
        digitalWrite(LED, LOW);
        delay(50);
    }

    // ledcSetup(1, 20000, 8);
    // ledcAttachPin(LED, 1);

    xTaskCreate(
        serialTask,   // タスク関数
        "serialTask", // タスク名
        2048,         // スタックサイズ（words）
        NULL,
        10, // 優先度
        NULL);

#if defined(MODE_RUNTIME)
    xTaskCreate(
        modeManagerTask,   // タスク関数
        "modeManagerTask", // タスク名
        4096,              // スタックサイズ（words）
        NULL,
        12, // 優先度
        NULL);

#elif defined(MODE_OUTPUT)
    xTaskCreate(Output_Task, "Output_Task", 2048, NULL, 11, NULL);

#elif defined(MODE_INPUT)
    xTaskCreate(Input_Task, "Input_Task", 1024, NULL, 4, NULL);

#elif defined(MODE_IO)
    xTaskCreate(IO_Task, "IO_Task", 2048, NULL, 11, NULL);

#elif defined(MODE_ROBOMAS)
    robomas_init();
    xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, NULL);
    xTaskCreate(PID_Task, "PID_Task", 2048, NULL, 11, NULL);

#elif defined(MODE_ROBOMAS_PLUS_OUTPUT)
    robomas_init();
    xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, NULL);
    xTaskCreate(Output_Task, "Output_Task", 2048, NULL, 8, NULL);

#elif defined(MODE_ROBOMAS_PLUS_INPUT)
    robomas_init();
    xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, NULL);
    xTaskCreate(Input_Task, "Input_Task", 1024, NULL, 4, NULL);

#elif defined(MODE_ROBOMAS_PLUS_IO)
    robomas_init();
    xTaskCreate(M3508_Task, "M3508_Task", 2048, NULL, 9, NULL);
    xTaskCreate(ROBOMAS_IO_Task, "ROBOMAS_IO_Task", 2048, NULL, 11, NULL);

#elif defined(MODE_DEBUG)
    xTaskCreate(PID_Task, "PID_Task", 2048, NULL, 11, NULL);
#endif
}

// ================= LOOP =================

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // メインループはなにもしない、処理はすべてFreeRTOSタスクで行う
}
