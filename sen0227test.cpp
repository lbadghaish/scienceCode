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
// Stepper (TB6600) + Heater pins from KiCad
// =====================================================
// KiCad nets:
// Pull(PWM)  -> GPIO8
// Direction+ -> GPIO9
// PumpEnable -> GPIO11 (active LOW enable)
// HeatSwitch -> GPIO10 (active LOW heater ON)

#define PIN_STEP_PULSE    8   // GPIO8  (pin 11)  TB6600 PUL
#define PIN_STEP_DIR      9   // GPIO9  (pin 12)  TB6600 DIR
#define PIN_STEP_ENABLE  11   // GPIO11 (pin 15)  TB6600 ENA (active LOW)

#define PIN_HEAT_SWITCH  10   // GPIO10 (pin 14)  active LOW = heater ON

// Stepper timing
#define MAX_STEP_DELAY_US    5000
#define MIN_STEP_DELAY_US     500
#define STEP_PULSE_WIDTH_US    10

static uint32_t speed_percent_to_delay_us(float speed_percent) {
    if (speed_percent <= 0.0f)  return 0;
    if (speed_percent > 100.0f) speed_percent = 100.0f;

    float p = speed_percent / 100.0f;
    float delay_f =
        (float)MAX_STEP_DELAY_US -
        p * (float)(MAX_STEP_DELAY_US - MIN_STEP_DELAY_US);

    if (delay_f < (float)MIN_STEP_DELAY_US) delay_f = (float)MIN_STEP_DELAY_US;
    return (uint32_t)delay_f;
}

void set_stepper_direction(bool up) {
    gpio_put(PIN_STEP_DIR, up ? 1 : 0);
}

void set_stepper_enabled(bool enabled) {
    // Active LOW enable per your notes
    gpio_put(PIN_STEP_ENABLE, enabled ? 0 : 1);
}

void stepper_move_steps(int32_t steps, float speed_percent, bool direction_up) {
    if (steps <= 0) return;

    uint32_t delay_us = speed_percent_to_delay_us(speed_percent);
    if (delay_us == 0) return;

    set_stepper_direction(direction_up);
    set_stepper_enabled(true);

    for (int32_t i = 0; i < steps; ++i) {
        gpio_put(PIN_STEP_PULSE, 1);
        sleep_us(STEP_PULSE_WIDTH_US);
        gpio_put(PIN_STEP_PULSE, 0);
        sleep_us(delay_us);
    }
}

// Heater: pull HeatSwitch LOW to turn on heater
void set_heater(bool on) {
    gpio_put(PIN_HEAT_SWITCH, on ? 0 : 1);
}

// =====================================================
// Pump H-Bridge (DRV8871) pins from KiCad
// =====================================================
// KiCad nets:
// MotorIn1 -> GPIO12
// MotorIn2 -> GPIO13

#define PIN_PUMP_IN1  12   // GPIO12 (pin 16) -> DRV8871 IN1
#define PIN_PUMP_IN2  13   // GPIO13 (pin 17) -> DRV8871 IN2

enum PumpMode {
    PUMP_COAST = 0,   // IN1=0 IN2=0
    PUMP_REVERSE,     // IN1=0 IN2=1
    PUMP_FORWARD,     // IN1=1 IN2=0
    PUMP_BRAKE        // IN1=1 IN2=1
};

// NOTE: I matched your truth table exactly:
// Forward  = IN1=1 IN2=0
// Reverse  = IN1=0 IN2=1
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

// Boolean helper (forward = true / reverse = false)
void pump_set_enabled(bool enabled, bool forward) {
    if (!enabled) {
        pump_set_mode(PUMP_COAST);
    } else {
        pump_set_mode(forward ? PUMP_FORWARD : PUMP_REVERSE);
    }
}

// =====================================================
// main
// =====================================================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Hello from Science PCB!\n");

    // ---- I2C init ----
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ---- Stepper + heater GPIO init ----
    gpio_init(PIN_STEP_ENABLE);
    gpio_init(PIN_STEP_DIR);
    gpio_init(PIN_STEP_PULSE);
    gpio_init(PIN_HEAT_SWITCH);

    gpio_set_dir(PIN_STEP_ENABLE, GPIO_OUT);
    gpio_set_dir(PIN_STEP_DIR, GPIO_OUT);
    gpio_set_dir(PIN_STEP_PULSE, GPIO_OUT);
    gpio_set_dir(PIN_HEAT_SWITCH, GPIO_OUT);

    set_stepper_enabled(false);
    gpio_put(PIN_STEP_PULSE, 0);
    set_heater(false);

    // ---- Pump H-bridge GPIO init ----
    gpio_init(PIN_PUMP_IN1);
    gpio_init(PIN_PUMP_IN2);
    gpio_set_dir(PIN_PUMP_IN1, GPIO_OUT);
    gpio_set_dir(PIN_PUMP_IN2, GPIO_OUT);
    pump_set_mode(PUMP_COAST);

    // Optional: scan I2C at startup
    i2c_scan();

    while (true) {
        float t = read_temperature();
        float h = read_humidity();
        printf("Temp: %.2f C, Hum: %.2f %%\n", t, h);

        // --- quick pump direction test (optional) ---
        // pump_set_enabled(true, true);   // forward
        // sleep_ms(1000);
        // pump_set_enabled(true, false);  // reverse
        // sleep_ms(1000);
        // pump_set_enabled(false, true);  // coast/off

        sleep_ms(1000);
    }
}
