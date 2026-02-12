#include "melectric_torque.h"
#include "robotkernel/helpers.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <algorithm>

using namespace std;
using namespace robotkernel;
using namespace robotkernel::helpers;
using namespace melectric;

MODULE_DEF(module_melectric_torque, melectric::torque_sensor)

static_assert(sizeof(torque_sensor_output_pd) == 103,
    "Process data struct must be exactly 103 bytes");

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

torque_sensor::torque_sensor(const char* name, const YAML::Node& node)
    : module_base(name, node)
    , trigger_base(name, node)
    , trigger(name, node)
{
    vcan_interface_     = get_as<string>(node, "vcan_interface", "vcan0");
    cal_slope_          = get_as<double>(node["calibration"], "slope", 99.93348);
    cal_offset_         = get_as<double>(node["calibration"], "offset", 92.565);
    torque_can_id_      = get_as<uint32_t>(node, "torque_can_id", 0x18FA8032);
    sensor_base_can_id_ = get_as<uint32_t>(node, "sensor_base_can_id", 0x18FA8100);
    sensor_count_       = min(get_as<int>(node, "sensor_count", 13), 13);

    memset(&pd_data_, 0, sizeof(pd_data_));
    for (int i = 0; i < 13; i++)
        sensor_stale_counter_[i] = UINT32_MAX;
}

torque_sensor::~torque_sensor()
{
    close_can_socket();
}

// ---------------------------------------------------------------------------
// SocketCAN helpers
// ---------------------------------------------------------------------------

void torque_sensor::open_can_socket()
{
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
        throw runtime_error("Failed to create CAN socket: " + string(strerror(errno)));

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, vcan_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        close(can_socket_);
        can_socket_ = -1;
        throw runtime_error("vCAN interface not found: " + vcan_interface_);
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(can_socket_);
        can_socket_ = -1;
        throw runtime_error("Failed to bind CAN socket: " + string(strerror(errno)));
    }

    // Non-blocking for tick()
    fcntl(can_socket_, F_SETFL, O_NONBLOCK);

    // CAN filter: torque ID + sensor ID range
    struct can_filter filters[2];
    filters[0].can_id   = torque_can_id_ | CAN_EFF_FLAG;
    filters[0].can_mask = CAN_EFF_MASK | CAN_EFF_FLAG;
    filters[1].can_id   = sensor_base_can_id_ | CAN_EFF_FLAG;
    filters[1].can_mask = 0x1FFFFF00 | CAN_EFF_FLAG;
    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters));
}

void torque_sensor::close_can_socket()
{
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
}

// ---------------------------------------------------------------------------
// CAN frame parsing
// ---------------------------------------------------------------------------

void torque_sensor::parse_torque_frame(const struct can_frame& frame)
{
    int16_t raw_torque = (int16_t)((frame.data[1] << 8) | frame.data[2]);
    pd_data_.torque_raw = raw_torque;
    pd_data_.torque_nm  = ((double)raw_torque - cal_offset_) / cal_slope_;
    pd_data_.torque_frame_count++;
    torque_stale_counter_ = 0;
}

void torque_sensor::parse_sensor_frame(const struct can_frame& frame, int sensor_idx)
{
    pd_data_.sensor[sensor_idx].x = (int16_t)((frame.data[0] << 8) | frame.data[1]);
    pd_data_.sensor[sensor_idx].y = (int16_t)((frame.data[2] << 8) | frame.data[3]);
    pd_data_.sensor[sensor_idx].z = (int16_t)((frame.data[4] << 8) | frame.data[5]);
    pd_data_.sensor_frame_count++;
    sensor_stale_counter_[sensor_idx] = 0;
}

// ---------------------------------------------------------------------------
// State transitions
// ---------------------------------------------------------------------------

void torque_sensor::set_state_init_2_preop()
{
    open_can_socket();
}

void torque_sensor::set_state_preop_2_safeop()
{
    output_pd_ = make_shared<process_data>(
        string(get_name()) + ".data.outputs.pd", sizeof(torque_sensor_output_pd));
    output_pd_provider_ = output_pd_->create_provider();
    robotkernel::add_device(output_pd_);
}

void torque_sensor::set_state_safeop_2_op()
{
    robotkernel::add_service(this, "tare",
        "Tare (zero) the torque sensor",
        [this](const YAML::Node&) { tare_pending_ = true; });
}

void torque_sensor::set_state_op_2_safeop()
{
    // Service remains registered but harmless in non-OP
}

void torque_sensor::set_state_safeop_2_preop()
{
    robotkernel::remove_device(output_pd_);
    output_pd_provider_.reset();
    output_pd_.reset();
}

void torque_sensor::set_state_preop_2_init()
{
    close_can_socket();
}

// ---------------------------------------------------------------------------
// Tare
// ---------------------------------------------------------------------------

void torque_sensor::tare()
{
    tare_pending_ = true;
}

// ---------------------------------------------------------------------------
// Cyclic callback
// ---------------------------------------------------------------------------

void torque_sensor::tick()
{
    // 1. Drain all available CAN frames (non-blocking)
    struct can_frame frame;
    ssize_t nbytes;
    while ((nbytes = ::read(can_socket_, &frame, sizeof(frame))) == sizeof(frame)) {
        consecutive_errors_ = 0;
        uint32_t id = frame.can_id & CAN_EFF_MASK;

        if (id == torque_can_id_ && frame.can_dlc == 8) {
            parse_torque_frame(frame);
        }
        else if (id >= sensor_base_can_id_ &&
                 id < sensor_base_can_id_ + (uint32_t)sensor_count_ &&
                 frame.can_dlc == 6) {
            parse_sensor_frame(frame, id - sensor_base_can_id_);
        }
    }

    // 2. Error handling: EAGAIN is normal, other errors count
    if (nbytes < 0 && errno != EAGAIN) {
        pd_data_.error_count++;
        consecutive_errors_++;
        if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
            throw runtime_error("CAN socket: " + to_string(consecutive_errors_) +
                                " consecutive read errors");
        }
    }

    // 3. Send tare command if pending
    if (tare_pending_.exchange(false)) {
        struct can_frame tare_frame;
        tare_frame.can_id  = torque_can_id_ | CAN_EFF_FLAG;
        tare_frame.can_dlc = 8;
        memset(tare_frame.data, 0, 8);
        tare_frame.data[0] = 0x89;
        ::write(can_socket_, &tare_frame, sizeof(tare_frame));
    }

    // 4. Update stale-data counters
    if (torque_stale_counter_ < UINT32_MAX)
        torque_stale_counter_++;
    pd_data_.torque_valid = (torque_stale_counter_ < TORQUE_STALE_THRESHOLD) ? 1 : 0;

    for (int i = 0; i < sensor_count_; i++) {
        if (sensor_stale_counter_[i] < UINT32_MAX)
            sensor_stale_counter_[i]++;
        if (sensor_stale_counter_[i] < SENSOR_STALE_THRESHOLD)
            pd_data_.sensors_valid_mask |= (1 << i);
        else
            pd_data_.sensors_valid_mask &= ~(1 << i);
    }

    // 5. Update process data
    auto pd_ptr = output_pd_->next(output_pd_provider_);
    memcpy(pd_ptr, &pd_data_, sizeof(pd_data_));
    output_pd_->push(output_pd_provider_);
}
