#include "inspection.h"

#include <math.h>
#include <string.h>

namespace inspection {

const char *name(State state) {
    switch (state) {
        case State::Boot:         return "BOOT";
        case State::Idle:         return "IDLE";
        case State::PartDetected: return "PART_DETECTED";
        case State::Settling:     return "SETTLING";
        case State::Measuring:    return "MEASURING";
        case State::Deciding:     return "DECIDING";
        case State::Actuating:    return "ACTUATING";
        case State::Clearing:     return "CLEARING";
        case State::Fault:        return "FAULT";
        case State::Estop:        return "ESTOP";
    }
    return "UNKNOWN";
}

const char *name(Fault fault) {
    switch (fault) {
        case Fault::None:               return "none";
        case Fault::MeasurementTimeout: return "measurement timeout, no echo from the sensor";
        case Fault::PartStuck:          return "part never left the fixture, possible jam";
    }
    return "unknown";
}

float median_of(const float *values, uint8_t count) {
    if (values == nullptr || count == 0) {
        return NAN;
    }

    float sorted[Machine::kMaxSamples];
    if (count > Machine::kMaxSamples) {
        count = Machine::kMaxSamples;
    }
    memcpy(sorted, values, sizeof(float) * count);

    for (uint8_t i = 1; i < count; i++) {
        float key = sorted[i];
        int16_t j = static_cast<int16_t>(i) - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }

    if (count % 2 == 1) {
        return sorted[count / 2];
    }
    return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
}

float height_from_distance(float sensor_offset_mm, float distance_mm) {
    return sensor_offset_mm - distance_mm;
}

bool verdict_pass(float height_mm, float target_mm, float tolerance_mm) {
    if (isnan(height_mm)) {
        return false;
    }
    return fabsf(height_mm - target_mm) <= tolerance_mm;
}

Machine::Machine(const Limits &limits) : limits_(limits) {
    if (limits_.measure_samples > kMaxSamples) {
        limits_.measure_samples = kMaxSamples;
    }
    if (limits_.measure_samples == 0) {
        limits_.measure_samples = 1;
    }
}

void Machine::begin(uint32_t now_ms) {
    state_ = State::Boot;
    previous_ = State::Boot;
    fault_ = Fault::None;
    entered_at_ = now_ms;
    last_sample_at_ = now_ms;
    sample_count_ = 0;
    counters_ = Counters();
}

void Machine::enter(State next, uint32_t now_ms) {
    previous_ = state_;
    state_ = next;
    entered_at_ = now_ms;
}

bool Machine::wants_sample(uint32_t now_ms) const {
    if (state_ != State::Measuring) {
        return false;
    }
    if (sample_count_ >= limits_.measure_samples) {
        return false;
    }
    return (now_ms - last_sample_at_) >= limits_.sample_gap_ms;
}

void Machine::push_sample(uint32_t now_ms, float distance_mm) {
    last_sample_at_ = now_ms;

    if (state_ != State::Measuring || sample_count_ >= limits_.measure_samples) {
        return;
    }
    if (isnan(distance_mm)) {
        return;
    }

    samples_[sample_count_++] = height_from_distance(limits_.sensor_offset_mm, distance_mm);
}

Outputs Machine::outputs_for_state() const {
    Outputs out;

    switch (state_) {
        case State::Deciding:
        case State::Actuating:
        case State::Clearing:
            out.gate = state_ == State::Clearing
                           ? Gate::Home
                           : (last_verdict_pass_ ? Gate::Accept : Gate::Reject);
            out.led_pass = last_verdict_pass_;
            out.led_fail = !last_verdict_pass_;
            break;

        case State::Fault:
        case State::Estop:
            out.gate = Gate::Home;
            out.led_fail = true;
            break;

        default:
            out.gate = Gate::Home;
            break;
    }

    return out;
}

Outputs Machine::step(const Inputs &in) {
    State before = state_;

    if (in.estop_active && state_ != State::Estop) {
        enter(State::Estop, in.now_ms);
        Outputs out = outputs_for_state();
        out.state_changed = true;
        return out;
    }

    bool report = false;
    uint32_t elapsed = in.now_ms - entered_at_;

    switch (state_) {
        case State::Boot:
            enter(State::Idle, in.now_ms);
            break;

        case State::Idle:
            if (in.part_present) {
                enter(State::PartDetected, in.now_ms);
            }
            break;

        case State::PartDetected:
            counters_.total++;
            enter(State::Settling, in.now_ms);
            break;

        case State::Settling:
            if (!in.part_present) {
                enter(State::Idle, in.now_ms);
            } else if (elapsed >= limits_.settle_ms) {
                sample_count_ = 0;
                last_sample_at_ = in.now_ms - limits_.sample_gap_ms;
                enter(State::Measuring, in.now_ms);
            }
            break;

        case State::Measuring:
            if (sample_count_ >= limits_.measure_samples) {
                last_height_mm_ = median_of(samples_, sample_count_);
                enter(State::Deciding, in.now_ms);
            } else if (elapsed > limits_.measure_timeout_ms) {
                fault_ = Fault::MeasurementTimeout;
                counters_.faults++;
                enter(State::Fault, in.now_ms);
            }
            break;

        case State::Deciding:
            last_verdict_pass_ =
                verdict_pass(last_height_mm_, limits_.target_mm, limits_.tolerance_mm);
            if (last_verdict_pass_) {
                counters_.passed++;
            } else {
                counters_.failed++;
            }
            report = true;
            enter(State::Actuating, in.now_ms);
            break;

        case State::Actuating:
            if (elapsed >= limits_.actuator_ms) {
                enter(State::Clearing, in.now_ms);
            }
            break;

        case State::Clearing:
            if (!in.part_present) {
                enter(State::Idle, in.now_ms);
            } else if (elapsed > limits_.clear_timeout_ms) {
                fault_ = Fault::PartStuck;
                counters_.faults++;
                enter(State::Fault, in.now_ms);
            }
            break;

        case State::Fault:
            if (in.reset_pressed) {
                fault_ = Fault::None;
                enter(State::Idle, in.now_ms);
            }
            break;

        case State::Estop:
            if (!in.estop_active && in.reset_pressed) {
                enter(State::Idle, in.now_ms);
            }
            break;
    }

    Outputs out = outputs_for_state();
    out.state_changed = state_ != before;

    if (report) {
        out.report_part = true;
        out.gate = last_verdict_pass_ ? Gate::Accept : Gate::Reject;
        out.led_pass = last_verdict_pass_;
        out.led_fail = !last_verdict_pass_;
        out.buzzer_hz = last_verdict_pass_ ? 1800 : 400;
        out.buzzer_ms = last_verdict_pass_ ? 90 : 300;
    }

    return out;
}

}
