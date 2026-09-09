#include "esphome/core/log.h"
#include "LD2410S.h"

#include <cstring>

namespace esphome
{
    namespace ld2410s
    {

        static const char* TAG = "ld2410s";

        static std::string format_bytes(const uint8_t* data, size_t length) {
            std::string out;
            char byte[4];
            for (size_t i = 0; i < length; i++) {
                snprintf(byte, sizeof(byte), "%02X", data[i]);
                if (i != 0) {
                    out += ':';
                }
                out += byte;
            }
            return out;
        }

        void LD2410S::setup() {
            this->set_config_mode(true);
            CmdFrameT read_fw_cmd = this->prepare_read_fw_cmd();
            this->send_command(read_fw_cmd);
            CmdFrameT read_config_cmd = this->prepare_read_config_cmd();
            this->send_command(read_config_cmd);
            this->set_config_mode(false);
        }

        void LD2410S::loop() {
            if (this->cmd_active) {
                return;
            }
            while (available()) {
                if (this->rx_pos >= RX_BUFFER_SIZE) {
                    // No frame was recognized while the buffer filled up. Drop everything before the
                    // next plausible frame start instead of writing past the end of the buffer.
                    this->resync_buffer();
                }
                const size_t match_pos = this->rx_pos;
                PackageType type = this->read_line(read(), this->rx_buffer, match_pos);
                this->rx_pos++;
                switch (type) {
                case PackageType::SHORT_DATA:
                case PackageType::TRESHOLD:
                    this->process_data_package(type, this->rx_buffer, match_pos);
                    this->rx_pos = 0;
                    break;
                case PackageType::ACK:
                    // Acks are only consumed by send_command(); one seen here belongs to a command we
                    // are no longer waiting for, so drop it and restart with an empty buffer.
                    ESP_LOGV(TAG, "Dropping unsolicited ack package");
                    this->rx_pos = 0;
                    break;
                default:
                    break;
                }
            }
        }

        void LD2410S::set_config_mode(bool enabled) {
            CmdFrameT start_cfg;
            start_cfg.header = CMD_FRAME_HEADER;
            start_cfg.command = enabled ? START_CONFIG_MODE_CMD : END_CONFIG_MODE_CMD;
            start_cfg.data_length = 0;
            if (enabled)
            {
                memcpy(&start_cfg.data[0], &START_CONFIG_MODE_VALUE, sizeof(START_CONFIG_MODE_VALUE));
                start_cfg.data_length += sizeof(START_CONFIG_MODE_VALUE);
            }

            start_cfg.footer = CMD_FRAME_FOOTER;
            this->send_command(start_cfg);
        }

        void LD2410S::apply_config() {
            this->status_set_warning("Sending command to sensor");
            this->set_config_mode(true);
            CmdFrameT apply_config_cmd = this->prepare_apply_config_cmd();
            this->send_command(apply_config_cmd);
            this->set_config_mode(false);
            this->status_clear_warning();
        }

        void LD2410S::start_auto_threshold_update() {
            this->status_set_warning("Sending command to sensor");
            this->set_config_mode(true);
            CmdFrameT threshold_update_cmd = this->prepare_threshold_cmd();
            this->send_command(threshold_update_cmd);
            this->set_config_mode(false);
            this->status_clear_warning();
        }

        CmdFrameT LD2410S::prepare_read_config_cmd() {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = READ_PARAMS_CMD;
            cmd_frame.data_length = 0;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MAX_DETECTION_VALUE, sizeof(CFG_MAX_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MAX_DETECTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MIN_DETECTION_VALUE, sizeof(CFG_MIN_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MIN_DETECTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_NO_DELAY_VALUE, sizeof(CFG_NO_DELAY_VALUE));
            cmd_frame.data_length += sizeof(CFG_NO_DELAY_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_STATUS_FREQ_VALUE, sizeof(CFG_STATUS_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_STATUS_FREQ_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_DISTANCE_FREQ_VALUE, sizeof(CFG_DISTANCE_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_DISTANCE_FREQ_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_RESPONSE_SPEED_VALUE, sizeof(CFG_RESPONSE_SPEED_VALUE));
            cmd_frame.data_length += sizeof(CFG_RESPONSE_SPEED_VALUE);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        CmdFrameT LD2410S::prepare_apply_config_cmd() {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = WRITE_PARAMS_CMD;
            cmd_frame.data_length = 0;

            Config to_save = this->new_config;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MAX_DETECTION_VALUE, sizeof(CFG_MAX_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MAX_DETECTION_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.max_dist, sizeof(to_save.max_dist));
            cmd_frame.data_length += sizeof(to_save.max_dist);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_MIN_DETECTION_VALUE, sizeof(CFG_MIN_DETECTION_VALUE));
            cmd_frame.data_length += sizeof(CFG_MIN_DETECTION_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.min_dist, sizeof(to_save.min_dist));
            cmd_frame.data_length += sizeof(to_save.min_dist);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_NO_DELAY_VALUE, sizeof(CFG_NO_DELAY_VALUE));
            cmd_frame.data_length += sizeof(CFG_NO_DELAY_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.delay, sizeof(to_save.delay));
            cmd_frame.data_length += sizeof(to_save.delay);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_STATUS_FREQ_VALUE, sizeof(CFG_STATUS_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_STATUS_FREQ_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.status_freq, sizeof(to_save.status_freq));
            cmd_frame.data_length += sizeof(to_save.status_freq);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_DISTANCE_FREQ_VALUE, sizeof(CFG_DISTANCE_FREQ_VALUE));
            cmd_frame.data_length += sizeof(CFG_DISTANCE_FREQ_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.dist_freq, sizeof(to_save.dist_freq));
            cmd_frame.data_length += sizeof(to_save.dist_freq);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &CFG_RESPONSE_SPEED_VALUE, sizeof(CFG_RESPONSE_SPEED_VALUE));
            cmd_frame.data_length += sizeof(CFG_RESPONSE_SPEED_VALUE);
            memcpy(&cmd_frame.data[cmd_frame.data_length], &to_save.resp_speed, sizeof(to_save.resp_speed));
            cmd_frame.data_length += sizeof(to_save.resp_speed);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        CmdFrameT LD2410S::prepare_threshold_cmd() {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = AUTO_UPDATE_THRESHOLD_CMD;
            cmd_frame.data_length = 0;

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_TRIGGER_VALUE, sizeof(THRESHOLD_TRIGGER_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_TRIGGER_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_RETENTION_VALUE, sizeof(THRESHOLD_RETENTION_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_RETENTION_VALUE);

            memcpy(&cmd_frame.data[cmd_frame.data_length], &THRESHOLD_TIME_VALUE, sizeof(THRESHOLD_TIME_VALUE));
            cmd_frame.data_length += sizeof(THRESHOLD_TIME_VALUE);

            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        CmdFrameT LD2410S::prepare_read_fw_cmd() {
            CmdFrameT cmd_frame;
            cmd_frame.header = CMD_FRAME_HEADER;
            cmd_frame.command = READ_FW_CMD;
            cmd_frame.data_length = 0;
            cmd_frame.footer = CMD_FRAME_FOOTER;
            return cmd_frame;
        }

        void LD2410S::send_command(CmdFrameT frame)
        {
            this->cmd_active = true;
            uint32_t start_millis = millis();
            uint8_t retry = 3;
            uint8_t cmd_buffer[64];

            while (retry)
            {
                frame.length = 0;
                uint16_t frame_data_bytes = frame.data_length + 2;
                // HEADER
                memcpy(&cmd_buffer[frame.length], &frame.header, sizeof(frame.header));
                frame.length += sizeof(frame.header);
                // SIZE
                memcpy(&cmd_buffer[frame.length], &frame_data_bytes, sizeof(frame.data_length));
                frame.length += sizeof(frame.data_length);
                // COMMAND
                memcpy(&cmd_buffer[frame.length], &frame.command, sizeof(frame.command));
                frame.length += sizeof(frame.command);
                // DATA
                for (uint16_t index = 0; index < frame.data_length; index++)
                {
                    memcpy(&cmd_buffer[frame.length], &frame.data[index], sizeof(frame.data[index]));
                    frame.length += sizeof(frame.data[index]);
                }
                // FOOTER
                memcpy(cmd_buffer + frame.length, &frame.footer, sizeof(frame.footer));
                frame.length += sizeof(frame.footer);
                // WRITE
                for (uint16_t index = 0; index < frame.length; index++)
                {
                    this->write_byte(cmd_buffer[index]);
                }

                this->flush();

                bool reply = false;

                while (!reply) {
                    uint8_t ack_buffer[RX_BUFFER_SIZE];
                    size_t last_pos = 0;
                    while (available()) {
                        if (last_pos >= RX_BUFFER_SIZE) {
                            last_pos = 0;
                        }
                        const size_t match_pos = last_pos;
                        PackageType type = this->read_line(read(), ack_buffer, match_pos);
                        last_pos++;
                        if (type == PackageType::ACK) {
                            reply = this->process_cmd_ack_package(ack_buffer, match_pos + 1);
                            last_pos = 0;
                        }
                    }
                    delay_microseconds_safe(1450);
                    if ((millis() - start_millis) > 1000)
                    {
                        start_millis = millis();
                        retry--;
                        break;
                    }
                }
                if (reply)
                {
                    retry = 0;
                }
            }
            this->cmd_active = false;
        }

        PackageType LD2410S::read_line(uint8_t data, uint8_t* buffer, size_t pos) {
            if (pos >= RX_BUFFER_SIZE) {
                // Caller failed to resync; never write past the end of the buffer.
                return PackageType::UNKNOWN;
            }
            buffer[pos] = data;

            if (pos >= 4) {
                if (memcmp(&buffer[pos - 3], &CMD_FRAME_FOOTER, sizeof(CMD_FRAME_FOOTER)) == 0) {
                    return PackageType::ACK;
                }
                else if (buffer[pos] == DATA_FRAME_FOOTER && buffer[pos - 4] == DATA_FRAME_HEADER) {
                    return PackageType::SHORT_DATA;
                }
                else if (memcmp(&buffer[pos - 3], &THRESHOLD_FOOTER, sizeof(THRESHOLD_FOOTER)) == 0
                    && this->find_frame_start(buffer, pos - 3, THRESHOLD_HEADER, nullptr)) {
                    return PackageType::TRESHOLD;
                }
            }
            return PackageType::UNKNOWN;
        }

        bool LD2410S::find_frame_start(const uint8_t* buffer, size_t footer_start, uint32_t header, size_t* start) {
            bool found = false;
            size_t candidate = 0;
            for (size_t i = 0; i + sizeof(header) <= footer_start; i++) {
                if (memcmp(&buffer[i], &header, sizeof(header)) == 0) {
                    candidate = i;
                    found = true;
                }
            }
            if (found && start != nullptr) {
                *start = candidate;
            }
            return found;
        }

        void LD2410S::resync_buffer() {
            const uint8_t data_header = static_cast<uint8_t>(DATA_FRAME_HEADER);
            const uint8_t cmd_header = static_cast<uint8_t>(CMD_FRAME_HEADER & 0xFF);
            const uint8_t threshold_header = static_cast<uint8_t>(THRESHOLD_HEADER & 0xFF);

            for (size_t i = 1; i < this->rx_pos; i++) {
                const uint8_t byte = this->rx_buffer[i];
                if (byte == data_header || byte == cmd_header || byte == threshold_header) {
                    const size_t remaining = this->rx_pos - i;
                    memmove(this->rx_buffer, &this->rx_buffer[i], remaining);
                    this->rx_pos = remaining;
                    ESP_LOGV(TAG, "Resynced RX buffer, discarded %u byte(s)", static_cast<unsigned>(i));
                    return;
                }
            }
            ESP_LOGV(TAG, "Resynced RX buffer, discarded %u byte(s)", static_cast<unsigned>(this->rx_pos));
            this->rx_pos = 0;
        }

        void LD2410S::process_config_read_ack(uint8_t* data) {
            int max_dist = this->read_int(data, 0, 4);
            int min_dist = this->read_int(data, 4, 4);
            int delay = this->read_int(data, 8, 4);
            int status_resp_freq = this->read_int(data, 12, 4);
            int dist_resp_freq = this->read_int(data, 16, 4);
            int resp_speed = this->read_int(data, 20, 4);
#ifdef USE_NUMBER
            this->max_distance_number->publish_state(max_dist);
            this->min_distance_number->publish_state(min_dist);
            this->no_delay_number->publish_state(delay);
            this->status_reporting_freq_number->publish_state(status_resp_freq / 10);
            this->distance_reporting_freq_number->publish_state(dist_resp_freq / 10);
#endif
#ifdef USE_SELECT
            this->response_speed_select->publish_state(resp_speed == 5 ? RESPONSE_SPEED_NORMAL : RESPONSE_SPEED_FAST);
#endif
            // Remember what the sensor actually reports, so that new_config starts from the
            // device's real settings. Without this, current_config stayed all-zero and
            // apply_config() wrote a zeroed configuration back to the sensor.
            this->current_config.max_dist = max_dist;
            this->current_config.min_dist = min_dist;
            this->current_config.delay = delay;
            this->current_config.status_freq = status_resp_freq;
            this->current_config.dist_freq = dist_resp_freq;
            this->current_config.resp_speed = resp_speed;
            this->new_config = this->current_config;
            ESP_LOGD(TAG, "Read config replay: max_dist=%d, min_dist=%d, delay=%d, status_resp_freq=%d, dist_resp_freq=%d, resp_speed=%d", max_dist, min_dist, delay, status_resp_freq, dist_resp_freq, resp_speed);
        }

        void LD2410S::process_read_fw_ack(uint8_t* data) {
            int major_v = static_cast<int>(data[0]);
            int minor_v = static_cast<int>(data[1]);
            int patch_v = static_cast<int>(data[2]);
            std::string version = "v" + std::to_string(major_v) + "." + std::to_string(minor_v) + "." + std::to_string(patch_v);
            for (auto& listener : this->listeners) {
                listener->on_fw_version(version);
            }
            ESP_LOGD(TAG, "Read firmware replay: %s", version.c_str());
        }

        bool LD2410S::process_cmd_ack_package(uint8_t* buffer, int len) {
            ESP_LOGV(TAG, "Ack buffer (%d bytes): %s", len, format_bytes(buffer, static_cast<size_t>(len)).c_str());
            CmdAckT ack = this->parse_ack(buffer, len);
            ESP_LOGV(TAG, "Ack command=%04X result=%s payload=%s", ack.command, YESNO(ack.result),
                format_bytes(ack.data, ack.length).c_str());
            int command_word = ack.command;
            bool result = ack.result;
            if (!result) {
                ESP_LOGW(TAG, "Command %x failed", command_word);
                return false;
            }
            else {
                ESP_LOGI(TAG, "Command %x success", command_word);
            }

            uint8_t* data = ack.data;

            switch (command_word) {
            case START_CONFIG_MODE_REPLAY:
                ESP_LOGD(TAG, "Config mode enabled");
                break;
            case END_CONFIG_MODE_REPLAY:
                ESP_LOGD(TAG, "Config mode disabled");
                break;
            case READ_PARAMS_REPLAY:
                this->process_config_read_ack(data);
                break;
            case WRITE_PARAMS_REPLAY:
                ESP_LOGD(TAG, "Write config replay processed");
                break;
            case READ_FW_REPLAY:
                this->process_read_fw_ack(data);
                break;
            default:
                ESP_LOGD(TAG, "Unknown replay: %x", command_word);
                break;
            }

            return true;
        }

        void LD2410S::process_short_data_package(uint8_t* data) {
            const bool presenceState = data[0] > 1;
            int distance = this->two_byte_to_int(data[1], data[2]);
            ESP_LOGV(TAG, "Short data decoded: state=%02X presence=%s distance=%d cm", data[0], YESNO(presenceState), distance);
            for (auto& listener : this->listeners) {
                listener->on_presence(presenceState);
                listener->on_distance(distance);
            }
        }

        void LD2410S::process_threshold_package(uint8_t* data) {
            int progress = this->two_byte_to_int(data[3], data[4]);
            for (auto& listener : this->listeners) {
                if (progress == 100) {
                    listener->on_threshold_progress(0);
                    listener->on_threshold_update(false);
                }
                else {
                    listener->on_threshold_progress(progress);
                    listener->on_threshold_update(true);
                }
            }
        }

        void LD2410S::process_data_package(PackageType type, uint8_t* buffer, size_t pos) {
            // `pos` is the index of the byte that completed the frame, i.e. where read_line() matched
            // the footer. Every field has to be read relative to that position, never from a fixed
            // offset, otherwise stale bytes of earlier/unrecognized data get decoded as sensor values.
            switch (type) {
            case PackageType::SHORT_DATA: {
                // read_line() matched buffer[pos - 4] == DATA_FRAME_HEADER and buffer[pos] ==
                // DATA_FRAME_FOOTER, so the payload is state, distance low, distance high at pos - 3.
                const uint8_t* frame = &buffer[pos - 4];
                ESP_LOGV(TAG, "Short data frame: %02X:%02X:%02X:%02X:%02X", frame[0], frame[1], frame[2], frame[3], frame[4]);
                this->process_short_data_package(&buffer[pos - 3]);
                break;
            }
            case PackageType::TRESHOLD: {
                // The footer occupies pos - 3 .. pos; the payload starts 4 bytes after the header.
                size_t start = 0;
                if (!this->find_frame_start(buffer, pos - 3, THRESHOLD_HEADER, &start)) {
                    ESP_LOGW(TAG, "Threshold footer without a matching header, dropping frame");
                    break;
                }
                if (start + 8 > pos) {
                    ESP_LOGW(TAG, "Threshold frame too short, dropping frame");
                    break;
                }
                ESP_LOGV(TAG, "Threshold frame at offset %u, length %u", static_cast<unsigned>(start), static_cast<unsigned>(pos - start + 1));
                this->process_threshold_package(&buffer[start + 4]);
                break;
            }
            default:
                ESP_LOGD(TAG, "Unexpected package type");
                break;
            }
        }

        float LD2410S::get_setup_priority() const
        {
            return setup_priority::HARDWARE;
        }

        CmdAckT LD2410S::parse_ack(uint8_t* buffer, size_t length) {
            CmdAckT result;
            bool found = false;
            size_t start = 0;
            for (size_t i = 0; i + sizeof(CMD_FRAME_HEADER) <= length; i++) {
                if (memcmp(&buffer[i], &CMD_FRAME_HEADER, sizeof(CMD_FRAME_HEADER)) == 0) {
                    start = i;
                    found = true;
                    break;
                }
            }
            if (!found) {
                ESP_LOGE(TAG, "Can't find cmd header");
                result.result = false;
                return result;
            }
            if (start + 10 > length) {
                ESP_LOGE(TAG, "Truncated ack package");
                result.result = false;
                return result;
            }
            int data_length = this->two_byte_to_int(buffer[start + 4], buffer[start + 5]);
            if (data_length > static_cast<int>(sizeof(result.data))) {
                data_length = static_cast<int>(sizeof(result.data));
            }
            if (start + 10 + static_cast<size_t>(data_length) > length) {
                data_length = static_cast<int>(length) - static_cast<int>(start) - 10;
            }
            result.length = data_length;
            int command_word = this->two_byte_to_int(buffer[start + 6], buffer[start + 7]);
            result.command = command_word;
            bool ack = buffer[start + 8] == 0x00 && buffer[start + 9] == 0x00;
            result.result = ack;
            // The payload starts 10 bytes after the header, wherever the header was found.
            memcpy(result.data, &buffer[start + 10], result.length);
            return result;
        }
    }
}