#pragma once

#include <cstddef>
#include <cstdint>

namespace serial_bridge::config {

    enum class LogOutputMode {
        kTerminal,
        kGraphical,
        kNone,
    };

    // ログ表示方法を選択する。
    // kTerminal: これまで通りターミナルに表示
    // kGraphical: ターミナルに簡易グラフィカル表示（ASCIIバー）
    // kNone    : ターミナルに表示しない
    inline constexpr LogOutputMode kLogOutputMode = LogOutputMode::kGraphical;
    inline constexpr int kGraphRxBarWidth = 24;
    inline constexpr int kGraphicalUiFrameMs = 30;

    inline constexpr std::size_t kTx16Num = 24;
    inline constexpr std::size_t kRx16Num = 17;
    inline constexpr uint8_t kStartByte = 0xAA;
    inline constexpr int kReadBufferSize = 512;
    inline constexpr int kUpdatePeriodMs = 5;

    inline constexpr const char *kParamVerbosePacketLog = "verbose_packet_log";
    inline constexpr bool kVerbosePacketLogDefault = false;

    inline constexpr const char *kParamStatusLogPeriodMs = "status_log_period_ms";
    inline constexpr int64_t kStatusLogPeriodMsDefault = 100;
    inline constexpr int64_t kStatusLogPeriodMsMin = 100;

} // namespace serial_bridge::config
