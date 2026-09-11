#pragma once

#include "../LD2410S.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome
{
    namespace ld2410s
    {
        class LD2410SSensor : public LD2410SListener, public Component, sensor::Sensor
        {
        public:
            void set_distance_sensor(sensor::Sensor *sensor) { this->distance_sensor = sensor; }
            void set_threshold_update_sensor(sensor::Sensor *sensor) { this->threshold_update_sensor = sensor; }
            void setup() override
            {
                // No threshold update is running at boot and nothing reports the progress until
                // one is started, so publish it now rather than leaving the entity unknown.
                this->on_threshold_progress(0);
            }
            void on_distance(int distance) override
            {
                this->publish(this->distance_sensor, this->distance_published, distance);
            }
            void on_threshold_progress(int progress) override
            {
                this->publish(this->threshold_update_sensor, this->threshold_update_published, progress);
            };

        private:
            // Only changes are published, to keep the sensor's repeated reports off the API, but
            // the first value has to go out even when it matches what the entity already holds -
            // otherwise a value that never changes leaves the entity "Unknown" in Home Assistant.
            static void publish(sensor::Sensor *sensor, bool &published, int value)
            {
                if (sensor == nullptr)
                {
                    return;
                }
                if (published && sensor->get_state() == value)
                {
                    return;
                }
                published = true;
                sensor->publish_state(value);
            }
            sensor::Sensor *distance_sensor{nullptr};
            bool distance_published{false};
            sensor::Sensor *threshold_update_sensor{nullptr};
            bool threshold_update_published{false};
        };
    }
}
