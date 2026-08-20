#pragma once

#include <stddef.h>
#include <stdint.h>

namespace inspection {

enum class State : uint8_t {
    Boot,
    Idle,
    PartDetected,
    Settling,
    Measuring,
    Deciding,
    Actuating,
    Clearing,
    Fault,
    Estop
};

enum class Fault : uint8_t {
    None,
    MeasurementTimeout,
    PartStuck
};

enum class Gate : uint8_t {
    Home,
    Accept,
    Reject
};

struct Limits {
    float target_mm = 45.0f;
    float tolerance_mm = 3.0f;
    float sensor_offset_mm = 120.0f;
    uint32_t settle_ms = 250;
    uint32_t sample_gap_ms = 30;
    uint32_t measure_timeout_ms = 2000;
    uint32_t actuator_ms = 600;
    uint32_t clear_timeout_ms = 5000;
    uint8_t measure_samples = 7;
};

struct Inputs {
    uint32_t now_ms = 0;
    bool part_present = false;
    bool estop_active = false;
    bool reset_pressed = false;
};

struct Outputs {
    Gate gate = Gate::Home;
    bool led_pass = false;
    bool led_fail = false;
    uint16_t buzzer_hz = 0;
    uint16_t buzzer_ms = 0;
    bool report_part = false;
    bool state_changed = false;
};

struct Counters {
    uint32_t total = 0;
    uint32_t passed = 0;
    uint32_t failed = 0;
    uint32_t faults = 0;
};

const char *name(State state);
const char *name(Fault fault);

float median_of(const float *values, uint8_t count);
float height_from_distance(float sensor_offset_mm, float distance_mm);
bool verdict_pass(float height_mm, float target_mm, float tolerance_mm);

class Machine {
public:
    static const uint8_t kMaxSamples = 15;

    explicit Machine(const Limits &limits);

    void begin(uint32_t now_ms);
    Outputs step(const Inputs &inputs);

    bool wants_sample(uint32_t now_ms) const;
    void push_sample(uint32_t now_ms, float distance_mm);

    State state() const { return state_; }
    State previous_state() const { return previous_; }
    Fault fault() const { return fault_; }
    float last_height_mm() const { return last_height_mm_; }
    bool last_verdict_pass() const { return last_verdict_pass_; }
    uint8_t samples_collected() const { return sample_count_; }
    uint32_t time_in_state(uint32_t now_ms) const { return now_ms - entered_at_; }
    const Counters &counters() const { return counters_; }

private:
    void enter(State next, uint32_t now_ms);
    Outputs outputs_for_state() const;

    Limits limits_;
    State state_ = State::Boot;
    State previous_ = State::Boot;
    Fault fault_ = Fault::None;
    uint32_t entered_at_ = 0;
    uint32_t last_sample_at_ = 0;

    float samples_[kMaxSamples] = {0};
    uint8_t sample_count_ = 0;

    float last_height_mm_ = 0.0f;
    bool last_verdict_pass_ = false;
    Counters counters_;
};

}
