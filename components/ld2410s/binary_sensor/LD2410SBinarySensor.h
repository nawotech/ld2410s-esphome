#pragma once

#include "../LD2410S.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
    namespace ld2410s {
        class LD2410SBinarySensor : public LD2410SListener, public Component, binary_sensor::BinarySensor
        {
        public:
            void set_presence_sensor(binary_sensor::BinarySensor* bsensor) { this->presence_bsensor = bsensor; };
            void set_threshold_update_sensor(binary_sensor::BinarySensor* bsensor) { this->threshold_update_bsensor = bsensor; };
            void setup() override {
                // A threshold update is never running at boot, and nothing else will report it
                // until one is started, so publish it now rather than leaving it unknown.
                this->on_threshold_update(false);
            };
            void on_presence(bool presence) override {
                this->publish(this->presence_bsensor, this->presence_published, presence);
            };
            void on_threshold_update(bool running) override {
                this->publish(this->threshold_update_bsensor, this->threshold_update_published, running);
            };

        private:
            // States are only published when they change, so that the sensor's repeated reports
            // don't flood the API. The first value still has to go out even when it matches the
            // default state, otherwise an entity whose value never changes (presence staying
            // false, for example) stays "Unknown" in Home Assistant forever.
            static void publish(binary_sensor::BinarySensor* bsensor, bool& published, bool state) {
                if (bsensor == nullptr) {
                    return;
                }
                if (published && bsensor->state == state) {
                    return;
                }
                published = true;
                bsensor->publish_state(state);
            };
            binary_sensor::BinarySensor* presence_bsensor{ nullptr };
            bool presence_published{ false };
            binary_sensor::BinarySensor* threshold_update_bsensor{ nullptr };
            bool threshold_update_published{ false };
        };
    }
}
