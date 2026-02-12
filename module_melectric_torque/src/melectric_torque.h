#pragma once

#include <robotkernel/module_base.h>
#include <robotkernel/trigger_base.h>
#include <robotkernel/trigger.h>
#include <robotkernel/process_data.h>
#include <string>
#include <atomic>
#include <cstdint>
#include <memory>

struct can_frame;

namespace melectric {

struct __attribute__((packed)) torque_sensor_output_pd {
    // Torque (updated at 500 Hz)
    int16_t  torque_raw;           // Raw CAN value (torque * 100)
    double   torque_nm;            // Calibrated torque in Nm

    // 13 sensors (updated at 100 Hz)
    struct __attribute__((packed)) {
        int16_t x;
        int16_t y;
        int16_t z;
    } sensor[13];                  // sensor[0]-sensor[12]

    // Status
    uint8_t  torque_valid;         // 1 = recently received, 0 = stale
    uint16_t sensors_valid_mask;   // Bit 0-12: one bit per sensor
    uint32_t torque_frame_count;   // Received torque frame counter
    uint32_t sensor_frame_count;   // Received sensor frame counter
    uint32_t error_count;          // CAN read errors
};

class torque_sensor : public robotkernel::module_base,
                      public robotkernel::trigger_base,
                      public robotkernel::trigger {
public:
    torque_sensor(const char* name, const YAML::Node& node);
    ~torque_sensor();

    // State transitions
    void set_state_init_2_preop() override;
    void set_state_preop_2_safeop() override;
    void set_state_safeop_2_op() override;
    void set_state_op_2_safeop() override;
    void set_state_safeop_2_preop() override;
    void set_state_preop_2_init() override;

    // Cyclic callback
    void tick() override;

    // Tare service
    void tare();

private:
    // Config
    std::string vcan_interface_;
    double cal_slope_;
    double cal_offset_;
    uint32_t torque_can_id_;
    uint32_t sensor_base_can_id_;
    int sensor_count_;

    // Runtime
    int can_socket_ = -1;
    torque_sensor_output_pd pd_data_;

    // Error tracking
    uint32_t consecutive_errors_ = 0;
    static constexpr uint32_t MAX_CONSECUTIVE_ERRORS = 100;

    // Stale-data counters (init: stale)
    uint32_t torque_stale_counter_ = UINT32_MAX;
    uint32_t sensor_stale_counter_[13];

    static constexpr uint32_t TORQUE_STALE_THRESHOLD = 5;    // 5 ms at 1 kHz
    static constexpr uint32_t SENSOR_STALE_THRESHOLD = 20;   // 20 ms at 1 kHz

    // Tare thread-safety
    std::atomic<bool> tare_pending_{false};

    // Process data devices
    std::shared_ptr<robotkernel::process_data> output_pd_;
    std::shared_ptr<robotkernel::pd_provider> output_pd_provider_;

    // Helpers
    void open_can_socket();
    void close_can_socket();
    void parse_torque_frame(const struct can_frame& frame);
    void parse_sensor_frame(const struct can_frame& frame, int sensor_idx);
};

} // namespace melectric
