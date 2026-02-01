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
// STEPPER MOTOR FSM (YOUR WORKING CODE)
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

// FSM state handler
void handle_motor_state() {
    switch (current_state) {
        case STATE_OFF:
            // Motor is stopped, do nothing
            printf("Motor State: OFF\n");
            break;
            
        case STATE_UP:
            printf("Motor State: UP - Moving motor up\n");
            set_stepper_direction(1);  // HIGH for one direction
            step_motor(2000, 200);  // 2000 steps, 200us pulse width
            break;
            
        case STATE_DOWN:
            printf("Motor State: DOWN - Moving motor down\n");
            set_stepper_direction(0);  // LOW for opposite direction
            step_motor(2000, 200);  // 2000 steps, 200us pulse width
            break;
    }
}

// Example function to change states (you can modify this based on your needs)
void update_stepper_state() {
    static uint32_t counter = 0;
    counter++;
    
    // Example: Cycle through states every few iterations
    // Replace this with your actual state transition logic
    // (buttons, sensors, serial commands, etc.)
    
    if (counter % 30 == 0) {
        current_state = STATE_UP;
    } else if (counter % 30 == 10) {
        current_state = STATE_DOWN;
    } else if (counter % 30 == 20) {
        current_state = STATE_OFF;
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

// NOTE: Matched truth table exactly:
// Forward  = IN1=1 IN2=0
// Reverse  = IN1=0 IN2=1
void pump_set_mode(enum PumpMode mode) {
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
// MAIN
// =====================================================
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Science PCB with FSM Stepper Control\n");
    printf("=====================================\n");

    // ---- I2C init ----
    i2c_init(I2C_PORT, 100 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ---- Stepper GPIO init (using your working code) ----
    init_stepper_pins();
    printf("Stepper motor initialized on GPIO %d (PUL) and GPIO %d (DIR)\n", PUL_PIN, DIR_PIN);

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

    printf("\nStarting main loop...\n\n");

    while (true) {
        // Read temperature and humidity
        float t = read_temperature();
        float h = read_humidity();
        printf("Temp: %.2f C, Hum: %.2f %% | ", t, h);

        // Update and handle stepper motor state
        update_stepper_state();
        handle_motor_state();

        // Optional: You can add pump or heater control here based on conditions
        // Example:
        // if (t > 30.0f) {
        //     set_heater(false);  // Turn off heater if too hot
        // } else if (t < 20.0f) {
        //     set_heater(true);   // Turn on heater if too cold
        // }

        sleep_ms(1000);
    }
}