#pragma once

#define PIN_PRESENCE      15
#define PIN_TRIG          16
#define PIN_ECHO          17
#define PIN_SERVO         18
#define PIN_LED_PASS      19
#define PIN_LED_FAIL      21
#define PIN_BUZZER        22
#define PIN_BUTTON        23
#define PIN_ESTOP         13

#define SERVO_HOME_DEG        90
#define SERVO_PASS_DEG        150
#define SERVO_REJECT_DEG      30

#define TARGET_HEIGHT_MM      45.0f
#define TOLERANCE_MM          3.0f

#define SETTLE_MS             250
#define MEASURE_SAMPLES       7
#define SAMPLE_GAP_MS         30
#define ACTUATOR_MS           600
#define CLEAR_TIMEOUT_MS      5000
#define MEASURE_TIMEOUT_MS    2000
#define DEBOUNCE_MS           40

#define ECHO_TIMEOUT_US       25000
#define SENSOR_OFFSET_MM      120.0f
