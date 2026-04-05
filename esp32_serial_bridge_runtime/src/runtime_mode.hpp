/*====================================================================
<runtime_mode.hpp>
・受信データ0番で指定するランタイムモード定義
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.
====================================================================*/

#pragma once

#include <stdint.h>

enum RuntimeMode : int16_t {
    RT_MODE_NONE = 0,
    RT_MODE_OUTPUT = 1,
    RT_MODE_INPUT = 2,
    RT_MODE_IO = 3,
    RT_MODE_ROBOMAS = 4,
    RT_MODE_ROBOMAS_PLUS_OUTPUT = 5,
    RT_MODE_ROBOMAS_PLUS_INPUT = 6,
    RT_MODE_ROBOMAS_PLUS_IO = 7,
    RT_MODE_DEBUG = 8,
};

RuntimeMode get_runtime_mode();