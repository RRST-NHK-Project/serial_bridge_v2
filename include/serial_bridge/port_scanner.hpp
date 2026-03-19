#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <string>

// skip_ports  : 既に使用中のポート（スキャンスレッドが管理）
// excluded_ports : 設定ファイルで除外指定されたポート
std::map<uint8_t, std::string> detect_serial_devices(
    const std::set<std::string> &skip_ports = {},
    const std::set<std::string> &excluded_ports = {});
