/*====================================================================
<runtime_mode.cpp>
・受信データ0番からランタイムモードを取得
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
====================================================================*/

#include "runtime_mode.hpp"
#include "frame_data.hpp"

RuntimeMode get_runtime_mode() {
    const int16_t raw_mode = Rx_16Data[0];

    if (raw_mode < RT_MODE_NONE || raw_mode > RT_MODE_DEBUG) {
        return RT_MODE_NONE;
    }

    return static_cast<RuntimeMode>(raw_mode);
}