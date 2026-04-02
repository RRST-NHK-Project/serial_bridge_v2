/*====================================================================
<defs.h>
・定数の定義ファイル

Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once
#include "config.hpp"
#include <Arduino.h>

// ================= ピンの定義 =================

// パルスカウンタの上限・下限の定義
#define COUNTER_H_LIM 32767
#define COUNTER_L_LIM -32768
#define PCNT_FILTER_VALUE 1023 // 0~1023, 1 = 12.5ns

#define ENC_PPR_SPEC 2048      // エンコーダの設定値
#define PPR (ENC_PPR_SPEC * 4) // 実効PPR（x4カウント）

#define DEG_PER_COUNT (360.0f / PPR)
#define HALF_PPR (PPR / 2)

// ピンの定義 //
// 状態表示LED
#define LED 0

// SDM15
#define SDM15_TX SDM15_UART_TX_PIN
#define SDM15_RX SDM15_UART_RX_PIN

// エンコーダ
#define ENC1_A 19
#define ENC1_B 21
#define ENC2_A 22
#define ENC2_B 23
#define ENC3_A 15
#define ENC3_B 16
#define ENC4_A 17
#define ENC4_B 18

// スイッチ
#define SW1 5
#define SW2 12
#define SW3 13
#define SW4 14
#define SW5 15
#define SW6 16
#define SW7 17
#define SW8 18

// CAN
#define CAN_RX 2
#define CAN_TX 4
