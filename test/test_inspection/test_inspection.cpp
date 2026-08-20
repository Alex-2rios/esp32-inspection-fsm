#include <unity.h>

#include "inspection.h"

using namespace inspection;

static Limits testLimits() {
    Limits limits;
    limits.target_mm = 45.0f;
    limits.tolerance_mm = 3.0f;
    limits.sensor_offset_mm = 120.0f;
    limits.settle_ms = 250;
    limits.sample_gap_ms = 30;
    limits.measure_timeout_ms = 2000;
    limits.actuator_ms = 600;
    limits.clear_timeout_ms = 5000;
    limits.measure_samples = 7;
    return limits;
}

struct Driver {
    Machine machine{testLimits()};
    uint32_t now = 0;
    Outputs last;

    Driver() { machine.begin(now); }

    Outputs advance(uint32_t ms, bool part, bool estop = false, bool reset = false) {
        now += ms;
        Inputs in;
        in.now_ms = now;
        in.part_present = part;
        in.estop_active = estop;
        in.reset_pressed = reset;
        last = machine.step(in);
        return last;
    }

    void feed_samples(float distance_mm, bool part = true) {
        for (uint8_t i = 0; i < 30; i++) {
            if (machine.state() != State::Measuring && machine.state() != State::Deciding) {
                break;
            }
            now += 30;
            if (machine.wants_sample(now)) {
                machine.push_sample(now, distance_mm);
            }
            Inputs in;
            in.now_ms = now;
            in.part_present = part;
            last = machine.step(in);
        }
    }

    void run_to_measuring() {
        advance(1, false);
        advance(1, true);
        advance(1, true);
        advance(300, true);
    }
};

void setUp(void) {}
void tearDown(void) {}

void test_median_picks_the_middle_value(void) {
    float values[5] = {44.0f, 46.0f, 45.0f, 45.5f, 44.5f};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, median_of(values, 5));
}

void test_median_ignores_a_wild_outlier(void) {
    float values[7] = {45.0f, 45.1f, 44.9f, 300.0f, 45.0f, 44.8f, 45.2f};
    TEST_ASSERT_FLOAT_WITHIN(0.11f, 45.0f, median_of(values, 7));
}

void test_mean_would_have_been_dragged_by_that_outlier(void) {
    float values[7] = {45.0f, 45.1f, 44.9f, 300.0f, 45.0f, 44.8f, 45.2f};
    float sum = 0;
    for (int i = 0; i < 7; i++) {
        sum += values[i];
    }
    TEST_ASSERT_TRUE((sum / 7.0f) > 80.0f);
}

void test_median_of_an_even_count_averages_the_middle_pair(void) {
    float values[4] = {10.0f, 20.0f, 30.0f, 40.0f};
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, median_of(values, 4));
}

void test_height_is_the_offset_minus_the_measured_distance(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, height_from_distance(120.0f, 75.0f));
}

void test_verdict_accepts_the_tolerance_boundary(void) {
    TEST_ASSERT_TRUE(verdict_pass(48.0f, 45.0f, 3.0f));
    TEST_ASSERT_TRUE(verdict_pass(42.0f, 45.0f, 3.0f));
}

void test_verdict_rejects_just_outside_the_boundary(void) {
    TEST_ASSERT_FALSE(verdict_pass(48.2f, 45.0f, 3.0f));
    TEST_ASSERT_FALSE(verdict_pass(41.8f, 45.0f, 3.0f));
}

void test_boot_moves_to_idle_on_the_first_step(void) {
    Driver d;
    TEST_ASSERT_EQUAL(State::Boot, d.machine.state());
    d.advance(1, false);
    TEST_ASSERT_EQUAL(State::Idle, d.machine.state());
}

void test_a_part_is_counted_once_and_only_once(void) {
    Driver d;
    d.run_to_measuring();
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().total);

    d.advance(10, true);
    d.advance(10, true);
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().total);
}

void test_a_part_pulled_out_while_settling_goes_back_to_idle(void) {
    Driver d;
    d.advance(1, false);
    d.advance(1, true);
    d.advance(1, true);
    TEST_ASSERT_EQUAL(State::Settling, d.machine.state());

    d.advance(50, false);
    TEST_ASSERT_EQUAL(State::Idle, d.machine.state());
}

void test_measuring_does_not_start_before_the_settle_time(void) {
    Driver d;
    d.advance(1, false);
    d.advance(1, true);
    d.advance(1, true);
    d.advance(100, true);
    TEST_ASSERT_EQUAL(State::Settling, d.machine.state());

    d.advance(200, true);
    TEST_ASSERT_EQUAL(State::Measuring, d.machine.state());
}

void test_samples_are_paced_by_the_gap(void) {
    Driver d;
    d.run_to_measuring();
    TEST_ASSERT_EQUAL(State::Measuring, d.machine.state());

    TEST_ASSERT_TRUE(d.machine.wants_sample(d.now));
    d.machine.push_sample(d.now, 75.0f);
    TEST_ASSERT_FALSE(d.machine.wants_sample(d.now + 10));
    TEST_ASSERT_TRUE(d.machine.wants_sample(d.now + 30));
}

void test_a_good_part_passes_and_opens_the_accept_gate(void) {
    Driver d;
    d.run_to_measuring();
    d.feed_samples(75.0f);

    TEST_ASSERT_TRUE(d.machine.last_verdict_pass());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 45.0f, d.machine.last_height_mm());
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().passed);
    TEST_ASSERT_EQUAL_UINT32(0, d.machine.counters().failed);
    TEST_ASSERT_EQUAL(Gate::Accept, d.last.gate);
    TEST_ASSERT_TRUE(d.last.led_pass);
}

void test_an_undersized_part_fails_and_opens_the_reject_gate(void) {
    Driver d;
    d.run_to_measuring();
    d.feed_samples(90.0f);

    TEST_ASSERT_FALSE(d.machine.last_verdict_pass());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 30.0f, d.machine.last_height_mm());
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().failed);
    TEST_ASSERT_EQUAL(Gate::Reject, d.last.gate);
    TEST_ASSERT_TRUE(d.last.led_fail);
}

void test_the_csv_line_is_emitted_exactly_once_per_part(void) {
    Driver d;
    d.run_to_measuring();

    int reports = 0;
    for (uint8_t i = 0; i < 20; i++) {
        d.now += 30;
        if (d.machine.wants_sample(d.now)) {
            d.machine.push_sample(d.now, 75.0f);
        }
        Inputs in;
        in.now_ms = d.now;
        in.part_present = true;
        Outputs out = d.machine.step(in);
        if (out.report_part) {
            reports++;
        }
    }

    TEST_ASSERT_EQUAL_INT(1, reports);
}

void test_readings_that_come_back_as_nan_are_not_counted(void) {
    Driver d;
    d.run_to_measuring();

    d.machine.push_sample(d.now, NAN);
    d.machine.push_sample(d.now + 30, NAN);
    TEST_ASSERT_EQUAL_UINT8(0, d.machine.samples_collected());

    d.machine.push_sample(d.now + 60, 75.0f);
    TEST_ASSERT_EQUAL_UINT8(1, d.machine.samples_collected());
}

void test_a_sensor_that_never_answers_ends_in_a_fault(void) {
    Driver d;
    d.run_to_measuring();

    d.advance(2100, true);

    TEST_ASSERT_EQUAL(State::Fault, d.machine.state());
    TEST_ASSERT_EQUAL(Fault::MeasurementTimeout, d.machine.fault());
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().faults);
    TEST_ASSERT_TRUE(d.last.led_fail);
    TEST_ASSERT_EQUAL(Gate::Home, d.last.gate);
}

void test_a_part_that_never_leaves_the_fixture_ends_in_a_fault(void) {
    Driver d;
    d.run_to_measuring();
    d.feed_samples(75.0f);

    d.advance(700, true);
    TEST_ASSERT_EQUAL(State::Clearing, d.machine.state());

    d.advance(5100, true);
    TEST_ASSERT_EQUAL(State::Fault, d.machine.state());
    TEST_ASSERT_EQUAL(Fault::PartStuck, d.machine.fault());
}

void test_a_fault_waits_for_an_operator_and_never_clears_itself(void) {
    Driver d;
    d.run_to_measuring();
    d.advance(2100, true);
    TEST_ASSERT_EQUAL(State::Fault, d.machine.state());

    d.advance(60000, false);
    TEST_ASSERT_EQUAL(State::Fault, d.machine.state());

    d.advance(1, false, false, true);
    TEST_ASSERT_EQUAL(State::Idle, d.machine.state());
    TEST_ASSERT_EQUAL(Fault::None, d.machine.fault());
}

void test_the_estop_interrupts_measuring_immediately(void) {
    Driver d;
    d.run_to_measuring();
    TEST_ASSERT_EQUAL(State::Measuring, d.machine.state());

    d.advance(1, true, true);
    TEST_ASSERT_EQUAL(State::Estop, d.machine.state());
    TEST_ASSERT_EQUAL(Gate::Home, d.last.gate);
    TEST_ASSERT_TRUE(d.last.led_fail);
}

void test_releasing_the_estop_is_not_enough_to_restart(void) {
    Driver d;
    d.run_to_measuring();
    d.advance(1, true, true);

    d.advance(100, false, false);
    TEST_ASSERT_EQUAL(State::Estop, d.machine.state());

    d.advance(1, false, false, true);
    TEST_ASSERT_EQUAL(State::Idle, d.machine.state());
}

void test_reset_while_the_estop_is_still_pressed_does_nothing(void) {
    Driver d;
    d.run_to_measuring();
    d.advance(1, true, true);

    d.advance(1, false, true, true);
    TEST_ASSERT_EQUAL(State::Estop, d.machine.state());
}

void test_a_full_run_of_three_parts_keeps_the_counters_straight(void) {
    Driver d;
    const float distances[3] = {75.0f, 90.0f, 76.0f};

    for (int i = 0; i < 3; i++) {
        d.run_to_measuring();
        d.feed_samples(distances[i]);
        d.advance(700, true);
        d.advance(10, false);
        TEST_ASSERT_EQUAL(State::Idle, d.machine.state());
    }

    TEST_ASSERT_EQUAL_UINT32(3, d.machine.counters().total);
    TEST_ASSERT_EQUAL_UINT32(2, d.machine.counters().passed);
    TEST_ASSERT_EQUAL_UINT32(1, d.machine.counters().failed);
    TEST_ASSERT_EQUAL_UINT32(0, d.machine.counters().faults);
}

int main(int, char **) {
    UNITY_BEGIN();

    RUN_TEST(test_median_picks_the_middle_value);
    RUN_TEST(test_median_ignores_a_wild_outlier);
    RUN_TEST(test_mean_would_have_been_dragged_by_that_outlier);
    RUN_TEST(test_median_of_an_even_count_averages_the_middle_pair);
    RUN_TEST(test_height_is_the_offset_minus_the_measured_distance);
    RUN_TEST(test_verdict_accepts_the_tolerance_boundary);
    RUN_TEST(test_verdict_rejects_just_outside_the_boundary);

    RUN_TEST(test_boot_moves_to_idle_on_the_first_step);
    RUN_TEST(test_a_part_is_counted_once_and_only_once);
    RUN_TEST(test_a_part_pulled_out_while_settling_goes_back_to_idle);
    RUN_TEST(test_measuring_does_not_start_before_the_settle_time);
    RUN_TEST(test_samples_are_paced_by_the_gap);

    RUN_TEST(test_a_good_part_passes_and_opens_the_accept_gate);
    RUN_TEST(test_an_undersized_part_fails_and_opens_the_reject_gate);
    RUN_TEST(test_the_csv_line_is_emitted_exactly_once_per_part);
    RUN_TEST(test_readings_that_come_back_as_nan_are_not_counted);

    RUN_TEST(test_a_sensor_that_never_answers_ends_in_a_fault);
    RUN_TEST(test_a_part_that_never_leaves_the_fixture_ends_in_a_fault);
    RUN_TEST(test_a_fault_waits_for_an_operator_and_never_clears_itself);

    RUN_TEST(test_the_estop_interrupts_measuring_immediately);
    RUN_TEST(test_releasing_the_estop_is_not_enough_to_restart);
    RUN_TEST(test_reset_while_the_estop_is_still_pressed_does_nothing);

    RUN_TEST(test_a_full_run_of_three_parts_keeps_the_counters_straight);

    return UNITY_END();
}
