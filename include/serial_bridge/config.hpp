#pragma once

#include <cstddef>
#include <cstdint>

namespace serial_bridge::config {

    inline constexpr std::size_t kTx16Num = 24;
    inline constexpr std::size_t kRx16Num = 17;
    inline constexpr uint8_t kStartByte = 0xAA;
    inline constexpr int kReadBufferSize = 128;
    inline constexpr int kUpdatePeriodMs = 20;

    inline constexpr const char *kParamVerbosePacketLog = "verbose_packet_log";
    inline constexpr bool kVerbosePacketLogDefault = false;

    inline constexpr const char *kParamStatusLogPeriodMs = "status_log_period_ms";
    inline constexpr int64_t kStatusLogPeriodMsDefault = 500;
    inline constexpr int64_t kStatusLogPeriodMsMin = 100;

} // namespace serial_bridge::config
