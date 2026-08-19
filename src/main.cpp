#include <Arduino.h>
#include <ESP32Servo.h>

#include "config.h"

enum State {
    STATE_BOOT,
    STATE_IDLE,
    STATE_PART_DETECTED,
    STATE_SETTLING,
    STATE_MEASURING,
    STATE_DECIDING,
    STATE_ACTUATING,
    STATE_CLEARING,
    STATE_FAULT,
    STATE_ESTOP
};

static const char *stateName(State s) {
    switch (s) {
        case STATE_BOOT:          return "BOOT";
        case STATE_IDLE:          return "IDLE";
        case STATE_PART_DETECTED: return "PART_DETECTED";
        case STATE_SETTLING:      return "SETTLING";
        case STATE_MEASURING:     return "MEASURING";
        case STATE_DECIDING:      return "DECIDING";
        case STATE_ACTUATING:     return "ACTUATING";
        case STATE_CLEARING:      return "CLEARING";
        case STATE_FAULT:         return "FAULT";
        case STATE_ESTOP:         return "ESTOP";
    }
    return "UNKNOWN";
}

Servo gate;

static State state = STATE_BOOT;
static unsigned long stateEnteredAt = 0;

static float samples[MEASURE_SAMPLES];
static uint8_t sampleIndex = 0;
static unsigned long lastSampleAt = 0;

static float lastHeight = 0;
static bool lastVerdictPass = false;
static const char *faultReason = "";

static uint32_t partsTotal = 0;
static uint32_t partsPassed = 0;
static uint32_t partsFailed = 0;
static uint32_t faults = 0;

static bool buttonStable = false;
static bool buttonRaw = false;
static unsigned long buttonChangedAt = 0;

static unsigned long elapsed() {
    return millis() - stateEnteredAt;
}

static void transition(State next) {
    Serial.printf("[%8lu] %-14s -> %-14s after %lu ms\n",
                  millis(), stateName(state), stateName(next), elapsed());
    state = next;
    stateEnteredAt = millis();
}

static bool partPresent() {
    return digitalRead(PIN_PRESENCE) == LOW;
}

static bool estopActive() {
    return digitalRead(PIN_ESTOP) == LOW;
}

static bool buttonPressed() {
    bool raw = digitalRead(PIN_BUTTON) == LOW;
    if (raw != buttonRaw) {
        buttonRaw = raw;
        buttonChangedAt = millis();
        return false;
    }
    if (millis() - buttonChangedAt < DEBOUNCE_MS) {
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

static float medianOf(float *values, uint8_t n) {
    float sorted[MEASURE_SAMPLES];
    memcpy(sorted, values, sizeof(float) * n);
    for (uint8_t i = 1; i < n; i++) {
        float key = sorted[i];
        int8_t j = i - 1;
        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = key;
    }
    return sorted[n / 2];
}

static void signalResult(bool pass) {
    digitalWrite(PIN_LED_PASS, pass ? HIGH : LOW);
    digitalWrite(PIN_LED_FAIL, pass ? LOW : HIGH);
    tone(PIN_BUZZER, pass ? 1800 : 400, pass ? 90 : 300);
}

static void clearSignals() {
    digitalWrite(PIN_LED_PASS, LOW);
    digitalWrite(PIN_LED_FAIL, LOW);
    noTone(PIN_BUZZER);
}

static void enterFault(const char *reason) {
    faultReason = reason;
    faults++;
    gate.write(SERVO_HOME_DEG);
    digitalWrite(PIN_LED_PASS, LOW);
    digitalWrite(PIN_LED_FAIL, HIGH);
    Serial.printf("[%8lu] fault: %s\n", millis(), reason);
    transition(STATE_FAULT);
}

static void reportPart() {
    Serial.printf("part,%lu,%s,%.1f,%.1f,%.1f,%lu,%lu\n",
                  partsTotal,
                  lastVerdictPass ? "pass" : "fail",
                  lastHeight,
                  TARGET_HEIGHT_MM,
                  TOLERANCE_MM,
                  partsPassed,
                  partsFailed);
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

    clearSignals();

    Serial.println();
    Serial.println("inspection station ready");
    Serial.printf("target %.1f mm, tolerance +/- %.1f mm\n", TARGET_HEIGHT_MM, TOLERANCE_MM);
    Serial.println("csv columns: part,index,verdict,measured_mm,target_mm,tolerance_mm,passed,failed");

    stateEnteredAt = millis();
    transition(STATE_IDLE);
}

void loop() {
    if (estopActive() && state != STATE_ESTOP) {
        gate.write(SERVO_HOME_DEG);
        clearSignals();
        digitalWrite(PIN_LED_FAIL, HIGH);
        transition(STATE_ESTOP);
        return;
    }

    switch (state) {

    case STATE_IDLE:
        if (partPresent()) {
            transition(STATE_PART_DETECTED);
        }
        break;

    case STATE_PART_DETECTED:
        partsTotal++;
        clearSignals();
        transition(STATE_SETTLING);
        break;

    case STATE_SETTLING:
        if (!partPresent()) {
            transition(STATE_IDLE);
        } else if (elapsed() >= SETTLE_MS) {
            sampleIndex = 0;
            lastSampleAt = 0;
            transition(STATE_MEASURING);
        }
        break;

    case STATE_MEASURING:
        if (elapsed() > MEASURE_TIMEOUT_MS) {
            enterFault("measurement timeout, no echo from the sensor");
            break;
        }
        if (millis() - lastSampleAt >= SAMPLE_GAP_MS) {
            lastSampleAt = millis();
            float mm = readDistanceMm();
            if (!isnan(mm)) {
                samples[sampleIndex++] = SENSOR_OFFSET_MM - mm;
            }
            if (sampleIndex >= MEASURE_SAMPLES) {
                lastHeight = medianOf(samples, MEASURE_SAMPLES);
                transition(STATE_DECIDING);
            }
        }
        break;

    case STATE_DECIDING:
        lastVerdictPass = fabsf(lastHeight - TARGET_HEIGHT_MM) <= TOLERANCE_MM;
        if (lastVerdictPass) {
            partsPassed++;
            gate.write(SERVO_PASS_DEG);
        } else {
            partsFailed++;
            gate.write(SERVO_REJECT_DEG);
        }
        signalResult(lastVerdictPass);
        reportPart();
        transition(STATE_ACTUATING);
        break;

    case STATE_ACTUATING:
        if (elapsed() >= ACTUATOR_MS) {
            gate.write(SERVO_HOME_DEG);
            transition(STATE_CLEARING);
        }
        break;

    case STATE_CLEARING:
        if (!partPresent()) {
            clearSignals();
            transition(STATE_IDLE);
        } else if (elapsed() > CLEAR_TIMEOUT_MS) {
            enterFault("part never left the fixture, possible jam");
        }
        break;

    case STATE_FAULT:
        if (buttonPressed()) {
            Serial.printf("[%8lu] fault %lu cleared by operator, was: %s\n",
                          millis(), faults, faultReason);
            clearSignals();
            gate.write(SERVO_HOME_DEG);
            transition(STATE_IDLE);
        }
        break;

    case STATE_ESTOP:
        if (!estopActive() && buttonPressed()) {
            clearSignals();
            transition(STATE_IDLE);
        }
        break;

    case STATE_BOOT:
        transition(STATE_IDLE);
        break;
    }
}
