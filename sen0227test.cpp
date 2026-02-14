#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <math.h>

// =====================================================
// I2C (Soil / SHT20)
// =====================================================
#define I2C_PORT    i2c1
#define I2C_SDA     2
#define I2C_SCL     3

#define SHT20_ADDR    0x40
#define TRIGGER_TEMP  0xF3
#define TRIGGER_HUM   0xF5

uint16_t sht20_read_raw(uint8_t command) {
    uint8_t raw[3];

    if (i2c_write_blocking(I2C_PORT, SHT20_ADDR, &command, 1, false) < 0)
        return 0xFFFF;

    sleep_ms(90);

    if (i2c_read_blocking(I2C_PORT, SHT20_ADDR, raw, 3, false) < 0)
        return 0xFFFF;

    uint16_t value = ((uint16_t)raw[0] << 8) | raw[1];
    value &= ~0x0003;
    return value;
}

float read_temperature(void) {
    uint16_t raw = sht20_read_raw(TRIGGER_TEMP);
    return (raw == 0xFFFF) ? -999.0f : (-46.85f + 175.72f * raw / 65536.0f);
}

float read_humidity(void) {
    uint16_t raw = sht20_read_raw(TRIGGER_HUM);
    if (raw == 0xFFFF) return -999.0f;
    float hum = -6.0f + 125.0f * raw / 65536.0f;
    if (hum > 100) hum = 100;
    if (hum < 0) hum = 0;
    return hum;
}

// =====================================================
// STEPPER MOTOR (non-blocking wrapper)
// =====================================================
#define PUL_PIN 8
#define DIR_PIN 9

typedef enum {
    STATE_OFF,
    STATE_UP,
    STATE_DOWN
} MotorState;

MotorState current_state = STATE_OFF;

void init_stepper_pins() {
    gpio_init(PUL_PIN);
    gpio_init(DIR_PIN);
    gpio_set_dir(PUL_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(PUL_PIN, 0);
    gpio_put(DIR_PIN, 0);
}

void step_motor(uint32_t steps, uint32_t delay_us) {
    for (uint32_t i = 0; i < steps; i++) {
        gpio_put(PUL_PIN, 1);
        sleep_us(delay_us);
        gpio_put(PUL_PIN, 0);
        sleep_us(delay_us);
    }
}

void set_stepper_direction(bool dir) {
    gpio_put(DIR_PIN, dir);
}

static uint32_t motor_steps_remaining = 0;
#define MOTOR_TOTAL_STEPS 2000
#define MOTOR_DELAY_US   200
#define MOTOR_CHUNK      50

void stepper_service_nonblocking(void) {
    if (current_state == STATE_OFF) {
        motor_steps_remaining = 0;
        return;
    }

    set_stepper_direction(current_state == STATE_UP);

    if (motor_steps_remaining == 0)
        motor_steps_remaining = MOTOR_TOTAL_STEPS;

    uint32_t chunk = (motor_steps_remaining > MOTOR_CHUNK) ? MOTOR_CHUNK : motor_steps_remaining;
    step_motor(chunk, MOTOR_DELAY_US);
    motor_steps_remaining -= chunk;
}

// =====================================================
// HEATER (MOSFET)  -- active LOW control pin
// =====================================================
#define PIN_HEAT_SWITCH 10

void init_heater(void) {
    gpio_init(PIN_HEAT_SWITCH);
    gpio_set_dir(PIN_HEAT_SWITCH, GPIO_OUT);
    gpio_put(PIN_HEAT_SWITCH, 1); // OFF default (active LOW)
}

void heater_set(bool on) {
    gpio_put(PIN_HEAT_SWITCH, on ? 0 : 1); // active LOW
}

// =====================================================
// Pump H-Bridge (DRV8871)
// =====================================================
#define PIN_PUMP_IN1 12
#define PIN_PUMP_IN2 13

typedef enum {
    PUMP_COAST = 0,
    PUMP_FORWARD,
    PUMP_REVERSE
} PumpMode;

void init_pump(void) {
    gpio_init(PIN_PUMP_IN1);
    gpio_init(PIN_PUMP_IN2);
    gpio_set_dir(PIN_PUMP_IN1, GPIO_OUT);
    gpio_set_dir(PIN_PUMP_IN2, GPIO_OUT);
    gpio_put(PIN_PUMP_IN1, 0);
    gpio_put(PIN_PUMP_IN2, 0);
}

void pump_set_mode(PumpMode mode) {
    switch (mode) {
        case PUMP_FORWARD:
            gpio_put(PIN_PUMP_IN1, 1);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
        case PUMP_REVERSE:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 1);
            break;
        default:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
    }
}

// =====================================================
// H-BRIDGE TEST (TIME-BASED, NON-BLOCKING)
// =====================================================
typedef enum {
    TEST_FWD,
    TEST_COAST_1,
    TEST_REV,
    TEST_COAST_2
} PumpTestState;

static PumpTestState pump_test_state = TEST_FWD;
static absolute_time_t pump_next_change;

void pump_test_service(void) {
    if (absolute_time_diff_us(get_absolute_time(), pump_next_change) > 0)
        return;

    switch (pump_test_state) {
        case TEST_FWD:
            pump_set_mode(PUMP_FORWARD);
            printf("PUMP: FORWARD\n");
            pump_test_state = TEST_COAST_1;
            pump_next_change = make_timeout_time_ms(5000);
            break;

        case TEST_COAST_1:
            pump_set_mode(PUMP_COAST);
            printf("PUMP: COAST\n");
            pump_test_state = TEST_REV;
            pump_next_change = make_timeout_time_ms(1000);
            break;

        case TEST_REV:
            pump_set_mode(PUMP_REVERSE);
            printf("PUMP: REVERSE\n");
            pump_test_state = TEST_COAST_2;
            pump_next_change = make_timeout_time_ms(5000);
            break;

        case TEST_COAST_2:
            pump_set_mode(PUMP_COAST);
            printf("PUMP: COAST\n");
            pump_test_state = TEST_FWD;
            pump_next_change = make_timeout_time_ms(1000);
            break;
    }
}

// =====================================================
// SERVO PWM (GPIO7 only)
// =====================================================
#define SERVO_PIN 7

// --- user-controlled variable (no input code needed) ---
static volatile float servo_angle_deg = 90.0f;   // set this anywhere: 0..180

// Servo timing (typical): 50 Hz, 1.0ms..2.0ms pulse range
#define SERVO_FREQ_HZ        50.0f
#define SERVO_MIN_PULSE_US  1000.0f
#define SERVO_MAX_PULSE_US  2000.0f
#define SERVO_MIN_ANGLE       0.0f
#define SERVO_MAX_ANGLE     180.0f

static uint servo_slice = 0;
static uint servo_chan  = 0;
static uint32_t servo_top = 0;

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    servo_chan  = pwm_gpio_to_channel(SERVO_PIN);

    // PWM freq = 125MHz / (div * (TOP+1))
    // Choose div = 64, compute TOP for 50Hz
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 64.0f);

    servo_top = (uint32_t)(125000000.0f / (64.0f * SERVO_FREQ_HZ)) - 1u;
    pwm_config_set_wrap(&cfg, servo_top);

    pwm_init(servo_slice, &cfg, true);
}

void servo_set_angle(float angle_deg) {
    angle_deg = clampf(angle_deg, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    float t = (angle_deg - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    float pulse_us = SERVO_MIN_PULSE_US + t * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    // counts_per_us = (125MHz/div)/1e6 = 125e6/64/1e6 = 1.953125 counts/us
    float counts_per_us = (125000000.0f / 64.0f) / 1000000.0f;
    uint16_t level = (uint16_t)(pulse_us * counts_per_us);

    if ((uint32_t)level > servo_top) level = (uint16_t)servo_top;
    pwm_set_chan_level(servo_slice, servo_chan, level);
}

// Keep servo updated periodically (cheap & safe)
static absolute_time_t next_servo_update;

void servo_service(void) {
    if (absolute_time_diff_us(get_absolute_time(), next_servo_update) > 0)
        return;

    servo_set_angle(servo_angle_deg);
    next_servo_update = make_timeout_time_ms(50); // 20 Hz updates
}

// =====================================================
// SERVO TEST (TIME-BASED, NON-BLOCKING)
// =====================================================
// Sweeps 0 -> 180 -> 0 with pauses.
// You can comment out servo_test_service() in main if you don't want the test.
typedef enum {
    SERVO_TEST_TO_0,
    SERVO_TEST_HOLD_0,
    SERVO_TEST_TO_90,
    SERVO_TEST_HOLD_90,
    SERVO_TEST_TO_180,
    SERVO_TEST_HOLD_180
} ServoTestState;

static ServoTestState servo_test_state = SERVO_TEST_TO_0;
static absolute_time_t servo_test_next;
static bool servo_test_enabled = true; // set false to disable test and use servo_angle_deg only

void servo_test_service(void) {
    if (!servo_test_enabled) return;

    if (absolute_time_diff_us(get_absolute_time(), servo_test_next) > 0)
        return;

    switch (servo_test_state) {
        case SERVO_TEST_TO_0:
            servo_angle_deg = 0.0f;
            printf("SERVO TEST: -> 0 deg\n");
            servo_test_state = SERVO_TEST_HOLD_0;
            servo_test_next = make_timeout_time_ms(1500);
            break;

        case SERVO_TEST_HOLD_0:
            servo_test_state = SERVO_TEST_TO_90;
            servo_test_next = make_timeout_time_ms(500);
            break;

        case SERVO_TEST_TO_90:
            servo_angle_deg = 90.0f;
            printf("SERVO TEST: -> 90 deg\n");
            servo_test_state = SERVO_TEST_HOLD_90;
            servo_test_next = make_timeout_time_ms(1500);
            break;

        case SERVO_TEST_HOLD_90:
            servo_test_state = SERVO_TEST_TO_180;
            servo_test_next = make_timeout_time_ms(500);
            break;

        case SERVO_TEST_TO_180:
            servo_angle_deg = 180.0f;
            printf("SERVO TEST: -> 180 deg\n");
            servo_test_state = SERVO_TEST_HOLD_180;
            servo_test_next = make_timeout_time_ms(1500);
            break;

        case SERVO_TEST_HOLD_180:
            servo_test_state = SERVO_TEST_TO_0;
            servo_test_next = make_timeout_time_ms(500);
            break;
    }
}

// =====================================================
// SYSTEM FSM (sensors printing on schedule)
// =====================================================
typedef enum {
    SYS_INIT,
    SYS_IDLE,
    SYS_READ_SENSORS
} SystemState;

static SystemState sys_state = SYS_INIT;

static absolute_time_t next_sensor_time;
#define SENSOR_PERIOD_MS 2000

// =====================================================
// MAIN
// =====================================================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Science PCB – FSM + Pump Test + Servo Test\n");

    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    init_stepper_pins();
    init_heater();
    init_pump();

    // Servo init (GPIO7)
    servo_init();
    servo_set_angle(servo_angle_deg);

    // start timers
    pump_next_change   = make_timeout_time_ms(1000);
    next_sensor_time   = make_timeout_time_ms(SENSOR_PERIOD_MS);
    next_servo_update  = make_timeout_time_ms(50);
    servo_test_next    = make_timeout_time_ms(500);

    while (true) {
        // keep stepper responsive
        stepper_service_nonblocking();

        // pump direction test
        pump_test_service();

        // servo PWM update
        servo_service();

        // servo sweep test (optional)
        servo_test_service();

        // sensor schedule
        switch (sys_state) {
            case SYS_INIT:
                current_state = STATE_OFF;
                heater_set(false);
                pump_set_mode(PUMP_COAST);
                sys_state = SYS_IDLE;
                break;

            case SYS_IDLE:
                if (absolute_time_diff_us(get_absolute_time(), next_sensor_time) <= 0)
                    sys_state = SYS_READ_SENSORS;
                break;

            case SYS_READ_SENSORS: {
                float t = read_temperature();
                float h = read_humidity();
                printf("Temp: %.2f C | Hum: %.2f %% | Servo: %.1f deg\n", t, h, servo_angle_deg);
                next_sensor_time = make_timeout_time_ms(SENSOR_PERIOD_MS);
                sys_state = SYS_IDLE;
                break;
            }
        }

        sleep_ms(5);
    }
}
