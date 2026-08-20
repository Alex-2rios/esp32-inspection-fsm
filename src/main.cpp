#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"
#include "inspection.h"

using inspection::Gate;
using inspection::Inputs;
using inspection::Limits;
using inspection::Machine;
using inspection::Outputs;
using inspection::State;

static Limits buildLimits() {
    Limits limits;
    limits.target_mm = TARGET_HEIGHT_MM;
    limits.tolerance_mm = TOLERANCE_MM;
    limits.sensor_offset_mm = SENSOR_OFFSET_MM;
    limits.settle_ms = SETTLE_MS;
    limits.sample_gap_ms = SAMPLE_GAP_MS;
    limits.measure_timeout_ms = MEASURE_TIMEOUT_MS;
    limits.actuator_ms = ACTUATOR_MS;
    limits.clear_timeout_ms = CLEAR_TIMEOUT_MS;
    limits.measure_samples = MEASURE_SAMPLES;
    return limits;
}

Servo gate;
static Machine machine(buildLimits());

static uint32_t lastTransitionAt = 0;
static State lastLoggedState = State::Boot;

static bool buttonStable = false;
static bool buttonRaw = false;
static uint32_t buttonChangedAt = 0;

static bool readResetButton(uint32_t now) {
    bool raw = digitalRead(PIN_BUTTON) == LOW;

    if (raw != buttonRaw) {
        buttonRaw = raw;
        buttonChangedAt = now;
        return false;
    }
    if (now - buttonChangedAt < DEBOUNCE_MS) {
        return false;
    }
    if (raw != buttonStable) {
        buttonStable = raw;
        return buttonStable;
    }
    return false;
}

static float readDistanceMm() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(3);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
    if (us == 0) {
        return NAN;
    }
    return (us * 0.343f) / 2.0f;
}

static void applyOutputs(const Outputs &out) {
    switch (out.gate) {
        case Gate::Accept: gate.write(SERVO_PASS_DEG); break;
        case Gate::Reject: gate.write(SERVO_REJECT_DEG); break;
        case Gate::Home:   gate.write(SERVO_HOME_DEG); break;
    }

    digitalWrite(PIN_LED_PASS, out.led_pass ? HIGH : LOW);
    digitalWrite(PIN_LED_FAIL, out.led_fail ? HIGH : LOW);

    if (out.buzzer_ms > 0) {
        tone(PIN_BUZZER, out.buzzer_hz, out.buzzer_ms);
    }
}

static void logTransition(uint32_t now) {
    Serial.printf("[%8lu] %-14s -> %-14s after %lu ms\n",
                  static_cast<unsigned long>(now),
                  inspection::name(lastLoggedState),
                  inspection::name(machine.state()),
                  static_cast<unsigned long>(now - lastTransitionAt));

    if (machine.state() == State::Fault) {
        Serial.printf("[%8lu] fault %lu: %s\n",
                      static_cast<unsigned long>(now),
                      static_cast<unsigned long>(machine.counters().faults),
                      inspection::name(machine.fault()));
    }

    lastLoggedState = machine.state();
    lastTransitionAt = now;
}

static void reportPart() {
    const inspection::Counters &counters = machine.counters();
    Serial.printf("part,%lu,%s,%.1f,%.1f,%.1f,%lu,%lu\n",
                  static_cast<unsigned long>(counters.total),
                  machine.last_verdict_pass() ? "pass" : "fail",
                  machine.last_height_mm(),
                  TARGET_HEIGHT_MM,
                  TOLERANCE_MM,
                  static_cast<unsigned long>(counters.passed),
                  static_cast<unsigned long>(counters.failed));
}

void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(PIN_PRESENCE, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_ESTOP, INPUT_PULLUP);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_LED_PASS, OUTPUT);
    pinMode(PIN_LED_FAIL, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    gate.setPeriodHertz(50);
    gate.attach(PIN_SERVO, 500, 2400);
    gate.write(SERVO_HOME_DEG);

    digitalWrite(PIN_LED_PASS, LOW);
    digitalWrite(PIN_LED_FAIL, LOW);

    uint32_t now = millis();
    machine.begin(now);
    lastTransitionAt = now;
    lastLoggedState = machine.state();

    Serial.println();
    Serial.println("inspection station ready");
    Serial.printf("target %.1f mm, tolerance +/- %.1f mm\n", TARGET_HEIGHT_MM, TOLERANCE_MM);
    Serial.println("csv columns: part,index,verdict,measured_mm,target_mm,tolerance_mm,passed,failed");
}

void loop() {
    uint32_t now = millis();

    if (machine.wants_sample(now)) {
        machine.push_sample(now, readDistanceMm());
    }

    Inputs in;
    in.now_ms = now;
    in.part_present = digitalRead(PIN_PRESENCE) == LOW;
    in.estop_active = digitalRead(PIN_ESTOP) == LOW;
    in.reset_pressed = readResetButton(now);

    Outputs out = machine.step(in);

    applyOutputs(out);

    if (out.report_part) {
        reportPart();
    }
    if (out.state_changed) {
        logTransition(now);
    }
}
