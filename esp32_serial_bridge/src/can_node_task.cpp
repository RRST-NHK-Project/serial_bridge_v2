#include "can_node_task.hpp"
#include "config.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>

namespace {

    constexpr uint8_t START_BYTE = 0xAA;
    constexpr uint32_t CAN_ID_CMD_BASE = 0x500;
    constexpr uint32_t CAN_ID_RSP_BASE = 0x600;
    constexpr uint32_t CAN_ID_MASK = 0x7FF;

    constexpr uint8_t CAN_PAYLOAD_PER_FRAME = 6;
    constexpr uint8_t CAN_FRAME_DATA_LEN = 8;

    constexpr uint8_t MAX_DATA_BYTES = ((Rx16NUM > Tx16NUM) ? Rx16NUM : Tx16NUM) * 2;
    constexpr uint8_t MAX_SERIAL_FRAME = 1 + 1 + 1 + MAX_DATA_BYTES + 1;

// 入力モードでは送信周期を短くする
#if defined(MODE_INPUT) || defined(MODE_ROBOMAS_PLUS_INPUT)
    constexpr uint32_t TX_PERIOD_MS = 20;
#else
    constexpr uint32_t TX_PERIOD_MS = 100;
#endif

    struct CanAssembly {
        bool active = false;
        uint8_t total_chunks = 0;
        uint8_t next_seq = 0;
        uint8_t data[MAX_SERIAL_FRAME] = {0};
        uint8_t length = 0;
        uint32_t last_update_ms = 0;
    };

    CanAssembly can_assembly;

    bool verifySerialFrame(const uint8_t *frame, uint8_t frame_len) {
        if (frame_len < 4) {
            return false;
        }
        if (frame[0] != START_BYTE) {
            return false;
        }

        const uint8_t data_len = frame[2];
        const uint8_t expected_len = static_cast<uint8_t>(1 + 1 + 1 + data_len + 1);
        if (frame_len != expected_len) {
            return false;
        }

        uint8_t checksum = 0;
        checksum ^= frame[1];
        checksum ^= frame[2];
        for (uint8_t i = 0; i < data_len; i++) {
            checksum ^= frame[3 + i];
        }
        return checksum == frame[3 + data_len];
    }

    void decodeRx16DataFromSerialFrame(const uint8_t *frame, uint8_t frame_len) {
        if (!verifySerialFrame(frame, frame_len)) {
            return;
        }

        const uint8_t target_id = frame[1];
        const uint8_t data_len = frame[2];
        if (target_id != DEVICE_ID) {
            return;
        }

        const uint8_t words = data_len / 2;
        const uint8_t copy_words = (words > Rx16NUM) ? Rx16NUM : words;

        for (uint8_t i = 0; i < copy_words; i++) {
            Rx_16Data[i] = static_cast<int16_t>((frame[3 + i * 2] << 8) | frame[3 + i * 2 + 1]);
        }
    }

    void buildTxSerialFrame(uint8_t *out_frame, uint8_t &out_len) {
        out_frame[0] = START_BYTE;
        out_frame[1] = DEVICE_ID;
        out_frame[2] = Tx16NUM * 2;

        uint8_t checksum = 0;
        checksum ^= out_frame[1];
        checksum ^= out_frame[2];

        for (int i = 0; i < Tx16NUM; i++) {
            out_frame[3 + i * 2] = static_cast<uint8_t>(Tx_16Data[i] >> 8);
            out_frame[3 + i * 2 + 1] = static_cast<uint8_t>(Tx_16Data[i] & 0xFF);
            checksum ^= out_frame[3 + i * 2];
            checksum ^= out_frame[3 + i * 2 + 1];
        }

        out_frame[3 + Tx16NUM * 2] = checksum;
        out_len = static_cast<uint8_t>(1 + 1 + 1 + (Tx16NUM * 2) + 1);
    }

    void sendSerialFrameOverCan(const uint8_t *serial_frame, uint8_t serial_len) {
        const uint8_t total_chunks =
            static_cast<uint8_t>((serial_len + CAN_PAYLOAD_PER_FRAME - 1) / CAN_PAYLOAD_PER_FRAME);

        for (uint8_t seq = 0; seq < total_chunks; seq++) {
            twai_message_t tx_msg = {};
            tx_msg.identifier = (CAN_ID_RSP_BASE + DEVICE_ID) & CAN_ID_MASK;
            tx_msg.extd = 0;
            tx_msg.rtr = 0;
            tx_msg.data_length_code = CAN_FRAME_DATA_LEN;

            tx_msg.data[0] = seq;
            tx_msg.data[1] = total_chunks;

            const uint8_t offset = seq * CAN_PAYLOAD_PER_FRAME;
            const uint8_t remaining = static_cast<uint8_t>(serial_len - offset);
            const uint8_t copy_len = remaining > CAN_PAYLOAD_PER_FRAME ? CAN_PAYLOAD_PER_FRAME : remaining;

            for (uint8_t i = 0; i < CAN_PAYLOAD_PER_FRAME; i++) {
                tx_msg.data[2 + i] = (i < copy_len) ? serial_frame[offset + i] : 0;
            }

            twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
        }
    }

    void resetAssembly() {
        can_assembly.active = false;
        can_assembly.total_chunks = 0;
        can_assembly.next_seq = 0;
        can_assembly.length = 0;
        can_assembly.last_update_ms = 0;
    }

    void flushAssemblyIfComplete() {
        if (!can_assembly.active || can_assembly.length < 4) {
            return;
        }

        const uint8_t expected_len = static_cast<uint8_t>(1 + 1 + 1 + can_assembly.data[2] + 1);
        if (expected_len > MAX_SERIAL_FRAME || can_assembly.length < expected_len) {
            return;
        }

        decodeRx16DataFromSerialFrame(can_assembly.data, expected_len);
        resetAssembly();
    }

    void pollCanRx() {
        twai_message_t rx_msg = {};
        while (twai_receive(&rx_msg, 0) == ESP_OK) {
            if (rx_msg.extd || rx_msg.rtr) {
                continue;
            }
            if (rx_msg.identifier != ((CAN_ID_CMD_BASE + DEVICE_ID) & CAN_ID_MASK)) {
                continue;
            }
            if (rx_msg.data_length_code != CAN_FRAME_DATA_LEN) {
                continue;
            }

            const uint8_t seq = rx_msg.data[0];
            const uint8_t total = rx_msg.data[1];
            if (total == 0 || total > 32) {
                continue;
            }

            const uint32_t now = millis();
            if (can_assembly.active && (now - can_assembly.last_update_ms > 50)) {
                resetAssembly();
            }

            if (seq == 0) {
                resetAssembly();
                can_assembly.active = true;
                can_assembly.total_chunks = total;
                can_assembly.next_seq = 0;
                can_assembly.length = 0;
            }

            if (!can_assembly.active) {
                continue;
            }
            if (total != can_assembly.total_chunks || seq != can_assembly.next_seq) {
                resetAssembly();
                continue;
            }

            for (uint8_t i = 0; i < CAN_PAYLOAD_PER_FRAME; i++) {
                if (can_assembly.length >= MAX_SERIAL_FRAME) {
                    resetAssembly();
                    break;
                }
                can_assembly.data[can_assembly.length++] = rx_msg.data[2 + i];
            }

            can_assembly.next_seq++;
            can_assembly.last_update_ms = now;

            if (can_assembly.next_seq >= can_assembly.total_chunks) {
                flushAssemblyIfComplete();
            }
        }
    }

    void pollCanTx() {
        static TickType_t last_tx = xTaskGetTickCount();
        const TickType_t now = xTaskGetTickCount();
        if (now - last_tx < pdMS_TO_TICKS(TX_PERIOD_MS)) {
            return;
        }

        uint8_t serial_frame[MAX_SERIAL_FRAME] = {0};
        uint8_t serial_len = 0;
        buildTxSerialFrame(serial_frame, serial_len);
        sendSerialFrameOverCan(serial_frame, serial_len);
        last_tx = now;
    }

} // namespace

void canNodeInit() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        while (1) {
            delay(100);
        }
    }

    if (twai_start() != ESP_OK) {
        while (1) {
            delay(100);
        }
    }

    resetAssembly();
}

void canNodeTask(void *) {
    canNodeInit();

    while (1) {
        pollCanRx();
        pollCanTx();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
