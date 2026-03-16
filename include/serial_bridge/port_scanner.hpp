#pragma once
#include <cstdint>
#include <map>
#include <set>
#include <string>

std::map<uint8_t, std::string> detect_serial_devices(const std::set<std::string> &skip_ports = {});
