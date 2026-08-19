#include <algorithm>

#include "esphome/core/log.h"
#include "midea_dishwasher.h"

namespace esphome::midea_dishwasher {

    void MideaDishwasher::setup() {
        tx_iface_buffer_.reserve(128);
        rx_iface_buffer_.reserve(128);
    }

    void MideaDishwasher::loop() {
        read_uart_(tx_iface_, tx_iface_buffer_);
        process_buffer_(tx_iface_buffer_, true);

        read_uart_(rx_iface_, rx_iface_buffer_);
        process_buffer_(rx_iface_buffer_, false);
    }

    void MideaDishwasher::read_uart_(uart::UARTComponent *uart, std::vector<uint8_t> &buffer) {
        static constexpr size_t READ_CHUNK_SIZE = 64;
        static constexpr size_t MAX_BUFFER_SIZE = 96;
        static constexpr size_t MAX_BYTES_PER_LOOP = MAX_BUFFER_SIZE;

        uint8_t chunk[READ_CHUNK_SIZE];

        // Snapshot the available byte count so a continuous stream cannot keep
        // this component in the read loop indefinitely. Any bytes received after
        // the snapshot remain in the UART ring buffer for the next loop call.
        size_t remaining = std::min(uart->available(), MAX_BYTES_PER_LOOP);

        while (remaining > 0 && buffer.size() < MAX_BUFFER_SIZE) {
            const size_t free_space = MAX_BUFFER_SIZE - buffer.size();
            const size_t count = std::min(std::min(remaining, READ_CHUNK_SIZE), free_space);
            if (!uart->read_array(chunk, count)) {
                break;
            }

            buffer.insert(buffer.end(), chunk, chunk + count);
            remaining -= count;
        }
    }

    void MideaDishwasher::process_buffer_(std::vector<uint8_t> &buffer, bool tx_packet) {
        while (buffer.size() >= 3) {
            if (buffer[0] != PREAMBLE) {
                buffer.erase(buffer.begin());
                continue;
            }

            uint8_t data_len = buffer[1];
            bool valid_length = tx_packet ? (data_len == TX_PACKET_LEN || data_len == TX_PACKET_LEN_VARIANT)
                                          : (data_len == RX_PACKET_LEN);
            if (!valid_length) {
                buffer.erase(buffer.begin());
                continue;
            }

            size_t packet_size = 3 + data_len;
            if (buffer.size() < packet_size)
                return;

            uint8_t checksum = 0;
            for (size_t i = 0; i < data_len; i++) 
                checksum += buffer[2 + i];
            
            if (buffer[packet_size - 1] != checksum)  {
                buffer.erase(buffer.begin());
                continue;
            }

            if (tx_packet) {
                if (detected_tx_packet_len_ != data_len) {
                    detected_tx_packet_len_ = data_len;
                    ESP_LOGI(TAG, "Detected status packet payload length: 0x%02X", data_len);
                }
                process_tx_packet_(buffer, data_len);
            } else {
                process_rx_packet_(buffer);
            }

            buffer.erase(buffer.begin(), buffer.begin() + packet_size);
        }
    }

    void MideaDishwasher::process_tx_packet_(std::vector<uint8_t> &buffer, uint8_t data_len) {
        uint8_t *data = buffer.data();
        
        publish_state_(error_, data[10] != 0);
        publish_state_(door_open_, (data[13] & 0x08) != 0);
        publish_state_(salt_low_, (data[13] & 0x10) != 0);
        publish_state_(rinse_aid_low_, (data[13] & 0x20) != 0);
        publish_state_(extra_dry_, (data[15] & 0x02) != 0);
        publish_state_(cycle_complete_, (data[12] & 0x07) == 5);
        publish_state_(paused_,
                       ((data[3] & 0x0f) == 3 && (data[3] >> 4) == 3) ||
                           ((data[3] & 0x0f) == 2 && (data[3] >> 4) == 2));
        publish_state_(running_, (data[3] >> 4) == 3);
        publish_state_(system_main_state_, data[3] >> 4);
        publish_state_(system_sub_state_, data[3] & 0x0f);
        publish_state_(program_phase_, data[12] & 0x07);
        publish_state_(operation_state_, data[4]);
        publish_state_(time_remaining_, data[5]);
        publish_state_(live_temperature_, data_len == TX_PACKET_LEN_VARIANT ? data[9] : (data[8] * 2) + 20);
        publish_state_(error_code_, data[10]);
        publish_state_(water_hardness_, data[12] >> 3);
        

        if (((data[3] >> 4) == 3) && !was_running_) {
            cycle_total_minutes_ = data[5];
            ESP_LOGD(TAG, "Cycle started, total minutes: %d", cycle_total_minutes_);
        }
        was_running_ = (data[3] >> 4) == 3;


        float progress;
        if (was_running_ && cycle_total_minutes_ > 0) {
            progress = (1.0f - (float)data[5] / (float)cycle_total_minutes_) * 100.0f;
            if (progress < 0) progress = 0;
            if (progress > 100) progress = 100;
        } else if ((data[12] & 0x07) == 5) {
            progress = 100.0f;
        } else {
            progress = 0.0f;
        }
        publish_state_(cycle_progress_, progress);

        const char* program = "Unknown";    
        switch (data[11]) {
            case 0:  program = "No Program"; break;
            case 1:  program = "AUTO"; break;
            case 2:  program = "P1 Intensive"; break;
            case 3:  program = "P2 Universal"; break;
            case 4:  program = "P3 ECO"; break;
            case 5:  program = "P4 Glass"; break;
            case 6:  program = "P5 90 min"; break;
            case 7:  program = "P6 Rapid"; break;
            case 16: program = "P7 Self-Clean"; break;
        }
        publish_state_(current_program_, std::string(program));

        const char* phase = "Unknown";
        switch (data[12] & 0x07) {
            case 0: phase = "Idle"; break;
            case 1: phase = "Pre-wash"; break;
            case 2: phase = "Main Wash"; break;
            case 3: phase = "Rinse"; break;
            case 4: phase = "Dry"; break;
            case 5: phase = "Complete"; break;
        }
        publish_state_(hr_current_program_phase_, std::string(phase));

        const char* op_state = "Unknown";
        switch (data[4]) {
            case 0:  op_state = "Idle"; break;
            case 1:  op_state = "Drain Pump"; break;
            case 6:  op_state = "Regeneration Valve"; break;
            case 11: op_state = "Heater + Spray Pump"; break;
            case 17: op_state = "Wait"; break;
            case 19: op_state = "Spray Pump"; break;
            case 20: op_state = "Heater + Spray Pump + Rinse-aid"; break;
            case 21: op_state = "Heater + Spray Pump + Detergent"; break;
            case 26: op_state = "Water Inlet"; break;
            case 31: op_state = "Regeneration Drain"; break;
            case 38: op_state = "Heater + Spray Pump"; break;
            case 48: op_state = "Pulsed Spray Pump"; break;
            case 49: op_state = "Program Select Display"; break;
            case 50: op_state = "End Program Display"; break;
            case 66: op_state = "Regeneration Valve"; break;
        }
        publish_state_(hr_system_operation_, std::string(op_state));

        const char* status = "Unknown";
        switch (data[3] >> 4) {
            case 0: status = "Powered Off"; break;
            case 4: status = "Error"; break;
            case 5: status = "Hardness Setting"; break;
            case 8: status = "Test Mode"; break;
            case 1:
                status = ((data[12] & 0x07) == 5) ? "Complete" : "Idle";
                break;
            case 2:
                status = ((data[3] & 0x0f) == 2) ? "Delay Start - Paused" : "Delay Start - Active";
                break;
            case 3: {
                uint8_t sub = data[3] & 0x0f;
                if (sub == 1) {
                    status = "Running - Transitioning";
                } else if (sub == 4) {
                    status = "Ending";
                } else if (sub == 3) {
                    status = "Paused";
                } else {
                    switch (data[12] & 0x07) {
                        case 1: status = "Running - Pre-wash"; break;
                        case 2: status = "Running - Main Wash"; break;
                        case 3: status = "Running - Rinse"; break;
                        case 4: status = "Running - Drying"; break;
                        default: status = "Running"; break;
                    }
                }
                break;
            }
        }
        publish_state_(hr_status_, std::string(status));
        
    }

    void MideaDishwasher::process_rx_packet_(std::vector<uint8_t> &buffer) {
        uint8_t *data = buffer.data();

        publish_state_(start_delay_, static_cast<uint16_t>(data[12]) | (static_cast<uint16_t>(data[13]) << 8));
        publish_state_(child_lock_, (data[15] & 0x08) != 0);
    }
}
