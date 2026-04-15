#pragma once

#include "serial_bridge/config.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace serial_bridge::graphical_ui {

    inline constexpr int kPanelWidth = 100;

    inline constexpr const char *kReset = "\033[0m";
    inline constexpr const char *kFgMuted = "\033[38;5;245m";
    inline constexpr const char *kFgTitle = "\033[38;5;45m";
    inline constexpr const char *kFgAccent = "\033[38;5;81m";
    inline constexpr const char *kFgGood = "\033[38;5;48m";
    inline constexpr const char *kFgWarn = "\033[38;5;214m";
    inline constexpr const char *kFgBad = "\033[38;5;203m";
    inline constexpr const char *kFgText = "\033[38;5;252m";

    inline std::string pad_or_trim(const std::string &text, int width) {
        if (width <= 0)
            return "";
        if (static_cast<int>(text.size()) >= width)
            return text.substr(0, static_cast<size_t>(width));
        return text + std::string(static_cast<size_t>(width - text.size()), ' ');
    }

    inline bool is_utf8_continuation(unsigned char c) {
        return (c & 0xC0u) == 0x80u;
    }

    // ANSIエスケープを可視幅に含めず、必要に応じて切り詰め/右側パディングする。
    inline std::string fit_ansi_text(const std::string &text, int width) {
        if (width <= 0)
            return "";

        std::string out;
        out.reserve(text.size() + 8);

        int visible = 0;
        size_t i = 0;
        while (i < text.size()) {
            // ANSI CSI sequence: ESC [ ... letter
            if (text[i] == '\033' && i + 1 < text.size() && text[i + 1] == '[') {
                const size_t start = i;
                i += 2;
                while (i < text.size()) {
                    const unsigned char ch = static_cast<unsigned char>(text[i]);
                    i++;
                    if ((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z')) {
                        break;
                    }
                }
                out.append(text, start, i - start);
                continue;
            }

            const unsigned char c = static_cast<unsigned char>(text[i]);
            size_t char_len = 1;
            if ((c & 0x80u) != 0u) {
                // UTF-8先頭バイトから連続バイトを取り込む
                size_t j = i + 1;
                while (j < text.size() &&
                       is_utf8_continuation(static_cast<unsigned char>(text[j]))) {
                    j++;
                }
                char_len = j - i;
            }

            if (visible >= width)
                break;

            out.append(text, i, char_len);
            visible++;
            i += char_len;
        }

        if (visible < width) {
            out += std::string(static_cast<size_t>(width - visible), ' ');
        }
        return out;
    }

    inline std::string panel_line(const std::string &content,
                                  const char *content_color = kFgText,
                                  const char *border_color = kFgMuted) {
        const int inner_width = kPanelWidth - 4;
        const std::string body = fit_ansi_text(content, inner_width);
        return std::string(border_color) + "| " + std::string(content_color) +
               body + std::string(border_color) + " |" + kReset;
    }

    inline std::string panel_border(const char *border_color = kFgMuted) {
        return std::string(border_color) + "+" +
               std::string(static_cast<size_t>(kPanelWidth - 2), '-') + "+" + kReset;
    }

    inline std::string colorize_scanner_line(const std::string &line) {
        if (line.find("detected") != std::string::npos || line.find("done") != std::string::npos)
            return std::string(kFgGood) + line + kReset;
        if (line.find("failed") != std::string::npos || line.find("no_frame") != std::string::npos)
            return std::string(kFgWarn) + line + kReset;
        if (line.find("skip") != std::string::npos)
            return std::string(kFgMuted) + line + kReset;
        return std::string(kFgAccent) + line + kReset;
    }

    inline std::string colorize_device_line(const std::string &line) {
        if (line.find("[OFF]") != std::string::npos)
            return std::string(kFgBad) + line + kReset;
        if (line.find("CHK 0") != std::string::npos &&
            line.find("TX(") != std::string::npos &&
            line.find("/0)") != std::string::npos)
            return std::string(kFgGood) + line + kReset;
        return std::string(kFgText) + line + kReset;
    }

    inline std::string spinner_frame(size_t tick) {
        static const char *frames[] = {
            "⠋", "⠙", "⠹", "⠸", "⠼", "⠴", "⠦", "⠧", "⠇", "⠏"};
        return frames[tick % (sizeof(frames) / sizeof(frames[0]))];
    }

    inline std::string header_pulse(size_t tick) {
        std::ostringstream oss;
        oss << spinner_frame(tick) << " syncing";
        return oss.str();
    }

    inline std::string node_anim_icon(bool is_on, size_t tick) {
        if (!is_on)
            return std::string(kFgBad) + "◼" + kReset;

        return std::string(kFgAccent) + spinner_frame(tick) + kReset;
    }

    struct DashboardState {
        std::string scanner_line{"idle"};
        std::string detected_line{"(none)"};
        std::map<uint8_t, std::string> node_lines;
        std::map<uint8_t, size_t> node_ticks;
        size_t render_tick{0};
        bool initialized{false};
    };

    inline DashboardState g_state;
    inline std::mutex g_mutex;
    inline std::atomic<bool> g_anim_running{false};
    inline std::thread g_anim_thread;

    inline void render_locked();

    inline void start_animation_thread_locked() {
        if (g_anim_running.load())
            return;

        g_anim_running = true;
        g_anim_thread = std::thread([]() {
            while (g_anim_running.load()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(std::max(16, serial_bridge::config::kGraphicalUiFrameMs)));

                std::lock_guard<std::mutex> lock(g_mutex);
                if (!g_state.initialized)
                    continue;
                render_locked();
            }
        });
    }

    inline bool enabled() {
        return serial_bridge::config::kLogOutputMode ==
               serial_bridge::config::LogOutputMode::kGraphical;
    }

    inline std::string format_device_id(uint8_t id) {
        std::ostringstream oss;
        oss << "0x" << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0') << static_cast<int>(id)
            << std::dec;
        return oss.str();
    }

    inline void render_locked() {
        if (!enabled())
            return;

        if (!g_state.initialized) {
            std::cout << "\033[?25l";
            g_state.initialized = true;
            start_animation_thread_locked();
        }

        std::cout << "\033[H\033[2J";
        g_state.render_tick++;
        std::cout << panel_border(kFgTitle) << "\n";
        std::cout << panel_line("serial_bridge live dashboard  ◉", kFgTitle, kFgTitle) << "\n";
        std::ostringstream mode_line;
        mode_line << "mode: graphical  |  style: modern-color  |  activity "
                  << std::string(kFgGood) << header_pulse(g_state.render_tick) << kReset;
        std::cout << panel_line(mode_line.str(), kFgMuted, kFgTitle) << "\n";
        std::cout << panel_border(kFgTitle) << "\n";
        std::cout << panel_line("SCAN", kFgAccent) << "\n";
        std::cout << panel_line(colorize_scanner_line(g_state.scanner_line), kFgText) << "\n";
        std::cout << panel_line("DETECTED", kFgAccent) << "\n";
        std::cout << panel_line(std::string(kFgGood) + g_state.detected_line + kReset, kFgText) << "\n";
        std::cout << panel_border() << "\n";
        std::cout << panel_line("DEVICES", kFgAccent) << "\n";

        if (g_state.node_lines.empty()) {
            std::cout << panel_line("(no active devices)", kFgMuted) << "\n";
        } else {
            for (const auto &[id, line] : g_state.node_lines) {
                const bool is_on = line.find("[ON ]") != std::string::npos;
                const size_t tick = g_state.node_ticks[id];
                std::ostringstream row;
                row << node_anim_icon(is_on, tick)
                    << "  " << format_device_id(id) << "  " << colorize_device_line(line);
                std::cout << panel_line(row.str(), kFgText) << "\n";
            }
        }

        std::cout << panel_border() << "\n";
        std::cout << std::string(kFgMuted)
                  << "tip: set LogOutputMode::kTerminal for classic logs"
                  << kReset << "\n";

        std::cout.flush();
    }

    inline void set_scanner_line(const std::string &line) {
        if (!enabled())
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_state.scanner_line = line;
        render_locked();
    }

    inline void set_detected_line(const std::string &line) {
        if (!enabled())
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_state.detected_line = line;
        render_locked();
    }

    inline void set_node_line(uint8_t id, const std::string &line) {
        if (!enabled())
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_state.node_lines[id] = line;
        const bool is_on = line.find("[ON ]") != std::string::npos;
        if (is_on) {
            g_state.node_ticks[id]++;
        } else {
            g_state.node_ticks[id] = 0;
        }
        render_locked();
    }

    inline void erase_node_line(uint8_t id) {
        if (!enabled())
            return;
        std::lock_guard<std::mutex> lock(g_mutex);
        g_state.node_lines.erase(id);
        g_state.node_ticks.erase(id);
        render_locked();
    }

    inline void shutdown() {
        if (!enabled())
            return;
        bool need_join = false;
        {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_anim_running = false;
            need_join = g_anim_thread.joinable();

            if (!g_state.initialized) {
                if (need_join) {
                    // join はロック外で実行
                } else {
                    return;
                }
            } else {
                std::cout << "\033[?25h" << std::flush;
                g_state.initialized = false;
            }
        }

        if (need_join)
            g_anim_thread.join();
    }

} // namespace serial_bridge::graphical_ui
