#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// =====================================================
// I2C (Soil / SHT20)  -- KiCad: I2C_SDA=GPIO2, I2C_SCL=GPIO3
// RP2040: GPIO2/3 are on I2C1
// =====================================================
#define I2C_PORT    i2c1
#define I2C_SDA     2   // GPIO2  (pin 4)
#define I2C_SCL     3   // GPIO3  (pin 5)

// SHT20
#define SHT20_ADDR    0x40
#define TRIGGER_TEMP  0xF3
#define TRIGGER_HUM   0xF5

// ----------------- I2C scan -----------------
void i2c_scan(void) {
    printf("\n=== I2C scan ===\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t dummy = 0;
        int ret = i2c_write_blocking(I2C_PORT, addr, &dummy, 1, true);
        if (ret > 0) {
            printf("  Found device at 0x%02X\n", addr);
        }
    }
    printf("Scan done.\n\n");
}

// ----------------- SHT20 helpers -----------------
uint16_t sht20_read_raw(uint8_t command) {
    uint8_t raw[3];

    int ret = i2c_write_blocking(I2C_PORT, SHT20_ADDR, &command, 1, false);
    if (ret < 0) {
        printf("I2C write error (cmd 0x%02X): %d\n", command, ret);
        return 0xFFFF;
    }

    sleep_ms(90);   // wait for conversion

    ret = i2c_read_blocking(I2C_PORT, SHT20_ADDR, raw, 3, false);
    if (ret < 0) {
        printf("I2C read error (cmd 0x%02X): %d\n", command, ret);
        return 0xFFFF;
    }

    uint16_t value = ((uint16_t)raw[0] << 8) | raw[1];
    value &= ~0x0003;   // clear status bits
    return value;
}

float read_temperature(void) {
    uint16_t raw = sht20_read_raw(TRIGGER_TEMP);
    if (raw == 0xFFFF) return -999.0f;
    return -46.85f + 175.72f * (float)raw / 65536.0f;
}

float read_humidity(void) {
    uint16_t raw = sht20_read_raw(TRIGGER_HUM);
    if (raw == 0xFFFF) return -999.0f;
    float hum = -6.0f + 125.0f * (float)raw / 65536.0f;
    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f) hum = 0.0f;
    return hum;
}

// =====================================================
// STEPPER MOTOR FSM (KEEPING YOUR WORKING CODE)
// =====================================================
// GPIO Pin Definitions
#define PUL_PIN 8   // PUL/PWM connected to GPIO 8 (TB6600 PUL)
#define DIR_PIN 9   // DIR connected to GPIO 9 (TB6600 DIR)

// FSM States
typedef enum {
    STATE_OFF,
    STATE_UP,
    STATE_DOWN
} MotorState;

// Global state variable
MotorState current_state = STATE_OFF;

// Function to initialize GPIO pins
void init_stepper_pins() {
    gpio_init(PUL_PIN);
    gpio_init(DIR_PIN);
    gpio_set_dir(PUL_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    // Initialize outputs to LOW
    gpio_put(PUL_PIN, 0);
    gpio_put(DIR_PIN, 0);
}

// Function to generate step pulses
void step_motor(uint32_t steps, uint32_t delay_us) {
    for (uint32_t i = 0; i < steps; i++) {
        gpio_put(PUL_PIN, 1);
        sleep_us(delay_us);
        gpio_put(PUL_PIN, 0);
        sleep_us(delay_us);
    }
}

// Function to set motor direction
void set_stepper_direction(bool dir) {
    gpio_put(DIR_PIN, dir);
}

// FSM state handler (your logic)
void handle_motor_state() {
    switch (current_state) {
        case STATE_OFF:
            // Motor is stopped, do nothing
            break;

        case STATE_UP:
            set_stepper_direction(1);    // HIGH for one direction
            step_motor(2000, 200);       // your working values
            break;

        case STATE_DOWN:
            set_stepper_direction(0);    // LOW for opposite direction
            step_motor(2000, 200);       // your working values
            break;
    }
}

// =====================================================
// HEATER CONTROL
// =====================================================
#define PIN_HEAT_SWITCH  10   // GPIO10 (pin 14)  active LOW = heater ON

// Heater: pull HeatSwitch LOW to turn on heater
void set_heater(bool on) {
    gpio_put(PIN_HEAT_SWITCH, on ? 0 : 1);
}

// =====================================================
// Pump H-Bridge (DRV8871) pins from KiCad
// =====================================================
#define PIN_PUMP_IN1  12   // GPIO12 (pin 16) -> DRV8871 IN1
#define PIN_PUMP_IN2  13   // GPIO13 (pin 17) -> DRV8871 IN2

typedef enum {
    PUMP_COAST = 0,   // IN1=0 IN2=0
    PUMP_REVERSE,     // IN1=0 IN2=1
    PUMP_FORWARD,     // IN1=1 IN2=0
    PUMP_BRAKE        // IN1=1 IN2=1
} PumpMode;

void pump_set_mode(PumpMode mode) {
    switch (mode) {
        case PUMP_COAST:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
        case PUMP_REVERSE:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 1);
            break;
        case PUMP_FORWARD:
            gpio_put(PIN_PUMP_IN1, 1);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
        case PUMP_BRAKE:
            gpio_put(PIN_PUMP_IN1, 1);
            gpio_put(PIN_PUMP_IN2, 1);
            break;
        default:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
    }
}

void pump_set_enabled(bool enabled, bool forward) {
    if (!enabled) {
        pump_set_mode(PUMP_COAST);
    } else {
        pump_set_mode(forward ? PUMP_FORWARD : PUMP_REVERSE);
    }
}

// =====================================================
// TOP-LEVEL SYSTEM FSM (non-testing, variable-driven)
// =====================================================
typedef enum {
    SYS_INIT = 0,
    SYS_IDLE,
    SYS_READ_SENSORS,
    SYS_APPLY_OUTPUTS
} SystemState;

static SystemState sys_state = SYS_INIT;

// ----- "User choices" are just variables for now -----
// (ROS will later set these variables. For now, you can hardcode them.)
typedef struct {
    // Stepper choice
    MotorState stepper_state;     // STATE_OFF / STATE_UP / STATE_DOWN

    // Pump choice
    bool pump_enabled;
    bool pump_forward;            // true=forward, false=reverse

    // Heater choice
    bool heater_on;

    // Sensor read rate
    uint32_t sensor_period_ms;    // how often to read sensors
} UserChoice;

static volatile UserChoice user_choice = {
    .stepper_state     = STATE_OFF,
    .pump_enabled      = false,
    .pump_forward      = true,
    .heater_on         = false,
    .sensor_period_ms  = 1000
};

// Latest sensor data (stored for ROS / logic)
typedef struct {
    float temperature_c;
    float humidity_pct;
    absolute_time_t last_read_time;
} SensorData;

static SensorData sensors = {
    .temperature_c  = -999.0f,
    .humidity_pct   = -999.0f
};

// Read sensors (blocking ~90ms inside sht20_read_raw; OK for now)
static void sensors_update(void) {
    sensors.temperature_c = read_temperature();
    sensors.humidity_pct  = read_humidity();
    sensors.last_read_time = get_absolute_time();
}

// =====================================================
// MAIN
// =====================================================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Science PCB - System FSM (variable-driven)\n");
    printf("=========================================\n");

    // ---- I2C init ----
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ---- Stepper GPIO init (your working code) ----
    init_stepper_pins();

    // ---- Heater GPIO init ----
    gpio_init(PIN_HEAT_SWITCH);
    gpio_set_dir(PIN_HEAT_SWITCH, GPIO_OUT);
    set_heater(false);

    // ---- Pump H-bridge GPIO init ----
    gpio_init(PIN_PUMP_IN1);
    gpio_init(PIN_PUMP_IN2);
    gpio_set_dir(PIN_PUMP_IN1, GPIO_OUT);
    gpio_set_dir(PIN_PUMP_IN2, GPIO_OUT);
    pump_set_mode(PUMP_COAST);

    // Optional: scan I2C at startup
    i2c_scan();

    // Timing for periodic sensor reads
    absolute_time_t next_sensor_time = make_timeout_time_ms(user_choice.sensor_period_ms);

    while (true) {
        switch (sys_state) {
            case SYS_INIT:
                // Put system into a known safe state
                current_state = STATE_OFF;                 // stepper off
                pump_set_mode(PUMP_COAST);                 // pump off
                set_heater(false);                         // heater off
                sys_state = SYS_IDLE;
                break;

            case SYS_IDLE:
                // Here check for updated commands (ROS/serial/etc.)
                // For now, "user_choice" is already the command source.

                // Sensor schedule
                if (absolute_time_diff_us(get_absolute_time(), next_sensor_time) <= 0) {
                    sys_state = SYS_READ_SENSORS;
                } else {
                    sys_state = SYS_APPLY_OUTPUTS;
                }
                break;

            case SYS_READ_SENSORS:
                sensors_update();

                // Print occasionally
                printf("Temp: %.2f C, Hum: %.2f %%\n", sensors.temperature_c, sensors.humidity_pct);

                // Schedule next read
                next_sensor_time = make_timeout_time_ms(user_choice.sensor_period_ms);

                sys_state = SYS_APPLY_OUTPUTS;
                break;

            case SYS_APPLY_OUTPUTS:
                // ----- Apply user choice to outputs -----

                // Stepper: map choice -> stepper state machine
                current_state = user_choice.stepper_state;
                handle_motor_state();  // uses working stepper logic

                // Pump H-bridge:
                pump_set_enabled(user_choice.pump_enabled, user_choice.pump_forward);

                // Heater:
                set_heater(user_choice.heater_on);

                // Go back to idle
                sys_state = SYS_IDLE;

                // Small sleep to reduce CPU use
                sleep_ms(10);
                break;

            default:
                sys_state = SYS_INIT;
                break;
        }
    }
}
