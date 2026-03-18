#include "serial_bridge/bridge_node.hpp"
#include "serial_bridge/port_scanner.hpp"
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto debug_node = std::make_shared<rclcpp::Node>("serial_bridge_status");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(debug_node);

    // デバイスID → ノード の管理マップ（スキャンスレッドと共有）
    std::map<uint8_t, std::shared_ptr<SerialBridgeNode>> node_map;
    std::set<std::string> known_ports;
    std::mutex map_mutex;
    std::atomic<bool> running{true};

    // バックグラウンドスキャンスレッド
    // 未検出のデバイスを定期的に探索し、新規デバイスが見つかれば動的にノードを追加する
    std::thread scanner([&]() {
        while (running && rclcpp::ok()) {
            // 既使用ポートを除外してスキャン（データ競合防止）
            std::set<std::string> skip;
            {
                std::lock_guard<std::mutex> lock(map_mutex);
                skip = known_ports;
            }

            auto devices = detect_serial_devices(skip);

            {
                std::lock_guard<std::mutex> lock(map_mutex);
                for (auto &[id, port] : devices) {
                    auto it = node_map.find(id);
                    if (it == node_map.end()) {
                        // 未知のデバイス: 新規ノードを作成
                        RCLCPP_INFO(debug_node->get_logger(),
                                    "New device found: ID=0x%02X port=%s",
                                    id, port.c_str());
                        auto node = std::make_shared<SerialBridgeNode>(id, port);
                        node_map[id] = node;
                        known_ports.insert(port);
                        executor.add_node(node);
                    } else if (!it->second->is_connected()) {
                        // 既知のデバイスが切断中に別ポートで再検出された: ノードを作り直す
                        // 注: is_connected() は atomic だが map_mutex とは同期しない。
                        // タイミングによって接続直後のノードを置き換える可能性がある（極めて稀）が、
                        // 新ノードが即座に再接続するため実害はない。
                        RCLCPP_INFO(debug_node->get_logger(),
                                    "Device ID=0x%02X reconnected on new port %s (was %s) — replacing node",
                                    id, port.c_str(), it->second->get_port().c_str());
                        executor.remove_node(it->second);
                        known_ports.erase(it->second->get_port());
                        auto node = std::make_shared<SerialBridgeNode>(id, port);
                        it->second = node;
                        known_ports.insert(port);
                        executor.add_node(node);
                    }
                    // 接続中かつ既知: 何もしない
                }
            }

            // 次のスキャンまで待機（100ms 刻みで running フラグを確認）
            constexpr int SCAN_INTERVAL_MS = 5000;
            for (int i = 0; i < SCAN_INTERVAL_MS / 100 && running && rclcpp::ok(); i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    executor.spin();

    running = false;
    scanner.join();

    rclcpp::shutdown();
    return 0;
}
