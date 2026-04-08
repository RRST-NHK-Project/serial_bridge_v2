#include "can_bridge_task.hpp"
#include "defs.hpp"
#include "frame_data.hpp"
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>

namespace {

    constexpr uint8_t START_BYTE = 0xAA;

    // CAN ID map (11-bit standard IDs)
    // Bridge -> node: 0x500 + target_device_id
    // Node -> bridge: 0x600 + source_device_id
    constexpr uint32_t CAN_ID_CMD_BASE = 0x500;
    constexpr uint32_t CAN_ID_RSP_BASE = 0x600;
    constexpr uint32_t CAN_ID_MASK = 0x7FF;

    // d0: seq, d1: total_chunks, d2..d7: payload
    constexpr uint8_t CAN_PAYLOAD_PER_FRAME = 6;
    constexpr uint8_t CAN_FRAME_DATA_LEN = 8;

    // Serial frame: START + ID + LEN + DATA + CHECKSUM
    // Bridge should be transport-agnostic, so keep a wider bound than mode-specific Rx/Tx sizes.
    // current CAN fragmentation limit: total_chunks <= 32 and payload/chunk == 6 bytes.
    constexpr uint8_t MAX_CAN_CHUNKS = 32;
    constexpr uint8_t MAX_SERIAL_FRAME = MAX_CAN_CHUNKS * CAN_PAYLOAD_PER_FRAME;
    constexpr uint8_t MAX_SERIAL_DATA_BYTES = MAX_SERIAL_FRAME - 4;

    enum RxState {
        WAIT_START,
        WAIT_ID,
        WAIT_LEN,
        WAIT_DATA,
        WAIT_CHECKSUM,
    };

    RxState serial_rx_state = WAIT_START;
    uint8_t serial_rx_id = 0;
    uint8_t serial_rx_len = 0;
    uint8_t serial_rx_buf[MAX_SERIAL_DATA_BYTES] = {0};
    uint8_t serial_rx_index = 0;
    uint8_t serial_rx_checksum = 0;

    struct CanAssembly {
        bool active = false;
        uint8_t source_id = 0;
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

    void sendSerialFrameOverCan(const uint8_t *serial_frame, uint8_t serial_len, uint8_t target_id) {
        if (serial_len == 0) {
            return;
        }

        const uint8_t total_chunks =
            static_cast<uint8_t>((serial_len + CAN_PAYLOAD_PER_FRAME - 1) / CAN_PAYLOAD_PER_FRAME);

        for (uint8_t seq = 0; seq < total_chunks; seq++) {
            twai_message_t tx_msg = {};
            tx_msg.identifier = (CAN_ID_CMD_BASE + target_id) & CAN_ID_MASK;
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

    void processCompleteSerialRx(uint8_t id, const uint8_t *payload, uint8_t payload_len, uint8_t checksum) {
        uint8_t serial_frame[MAX_SERIAL_FRAME] = {0};
        const uint8_t serial_len = static_cast<uint8_t>(1 + 1 + 1 + payload_len + 1);

        serial_frame[0] = START_BYTE;
        serial_frame[1] = id;
        serial_frame[2] = payload_len;
        for (uint8_t i = 0; i < payload_len; i++) {
            serial_frame[3 + i] = payload[i];
        }
        serial_frame[3 + payload_len] = checksum;

        if (!verifySerialFrame(serial_frame, serial_len)) {
            return;
        }

        sendSerialFrameOverCan(serial_frame, serial_len, id);
    }

    void pollSerialRxAndBridgeToCan() {
        while (Serial.available()) {
            const uint8_t b = static_cast<uint8_t>(Serial.read());

            switch (serial_rx_state) {
            case WAIT_START:
                if (b == START_BYTE) {
                    serial_rx_state = WAIT_ID;
                }
                break;

            case WAIT_ID:
                serial_rx_id = b;
                serial_rx_checksum = b;
                serial_rx_state = WAIT_LEN;
                break;

            case WAIT_LEN:
                serial_rx_len = b;
                serial_rx_checksum ^= b;

                if (serial_rx_len > MAX_SERIAL_DATA_BYTES) {
                    serial_rx_state = WAIT_START;
                } else {
                    serial_rx_index = 0;
                    serial_rx_state = (serial_rx_len == 0) ? WAIT_CHECKSUM : WAIT_DATA;
                }
                break;

            case WAIT_DATA:
                serial_rx_buf[serial_rx_index++] = b;
                serial_rx_checksum ^= b;
                if (serial_rx_index >= serial_rx_len) {
                    serial_rx_state = WAIT_CHECKSUM;
                }
                break;

            case WAIT_CHECKSUM:
                if (serial_rx_checksum == b) {
                    processCompleteSerialRx(serial_rx_id, serial_rx_buf, serial_rx_len, b);
                }
                serial_rx_state = WAIT_START;
                break;
            }
        }
    }

    void resetCanAssembly() {
        can_assembly.active = false;
        can_assembly.source_id = 0;
        can_assembly.total_chunks = 0;
        can_assembly.next_seq = 0;
        can_assembly.length = 0;
        can_assembly.last_update_ms = 0;
    }

    void tryFlushCanAssemblyToSerial() {
        if (!can_assembly.active || can_assembly.length < 4) {
            return;
        }

        const uint8_t frame_len = static_cast<uint8_t>(1 + 1 + 1 + can_assembly.data[2] + 1);
        if (frame_len > MAX_SERIAL_FRAME || can_assembly.length < frame_len) {
            return;
        }

        if (verifySerialFrame(can_assembly.data, frame_len)) {
            Serial.write(can_assembly.data, frame_len);
        }

        resetCanAssembly();
    }

    void handleCanFragment(uint8_t source_id, const twai_message_t &msg) {
        if (msg.data_length_code != CAN_FRAME_DATA_LEN) {
            return;
        }

        const uint8_t seq = msg.data[0];
        const uint8_t total = msg.data[1];
        if (total == 0 || total > 32) {
            return;
        }

        const uint32_t now = millis();
        if (can_assembly.active && (now - can_assembly.last_update_ms > 50)) {
            resetCanAssembly();
        }

        if (seq == 0) {
            resetCanAssembly();
            can_assembly.active = true;
            can_assembly.source_id = source_id;
            can_assembly.total_chunks = total;
            can_assembly.next_seq = 0;
            can_assembly.length = 0;
        }

        if (!can_assembly.active || can_assembly.source_id != source_id) {
            return;
        }

        if (seq != can_assembly.next_seq || total != can_assembly.total_chunks) {
            resetCanAssembly();
            return;
        }

        for (uint8_t i = 0; i < CAN_PAYLOAD_PER_FRAME; i++) {
            if (can_assembly.length >= MAX_SERIAL_FRAME) {
                resetCanAssembly();
                return;
            }
            can_assembly.data[can_assembly.length++] = msg.data[2 + i];
        }

        can_assembly.next_seq++;
        can_assembly.last_update_ms = now;

        if (can_assembly.next_seq >= can_assembly.total_chunks) {
            tryFlushCanAssemblyToSerial();
        }
    }

    void pollCanRxAndBridgeToSerial() {
        twai_message_t rx_msg = {};
        while (twai_receive(&rx_msg, 0) == ESP_OK) {
            if (rx_msg.extd) {
                continue;
            }
            if (rx_msg.identifier < CAN_ID_RSP_BASE || rx_msg.identifier > (CAN_ID_RSP_BASE + 0xFF)) {
                continue;
            }

            const uint8_t source_id = static_cast<uint8_t>(rx_msg.identifier - CAN_ID_RSP_BASE);
            handleCanFragment(source_id, rx_msg);
        }
    }

} // namespace

void canBridgeInit() {
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
}

void canBridgeTask(void *) {
    canBridgeInit();
    resetCanAssembly();

    while (1) {
        pollSerialRxAndBridgeToCan();
        pollCanRxAndBridgeToSerial();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
