#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ================= I2C (Soil / SHT20) =================

// On your PCB: GPIO2 = SDA, GPIO3 = SCL
// NOTE: on RP2040, GPIO2/3 belong to I2C1
#define I2C_PORT    i2c1
#define I2C_SDA     2     // GPIO2
#define I2C_SCL     3     // GPIO3

// SHT20 address & commands
#define SHT20_ADDR   0x40
#define TRIGGER_TEMP 0xF3
#define TRIGGER_HUM  0xF5

// ------------- I2C scan -------------
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

// ------------- SHT20 helpers -------------
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


// ================= Stepper + Heater =================

// From schematic notes:
//
// GPIO11 – pull low to enable pump (TB6600 ENA-)
// GPIO10 – pull low to enable heater (HeatSwitch gate driver)
// GPIO9  – + = one direction, pull low for other (TB6600 DIR)
// GPIO8  – pulse: does it move or not (TB6600 PUL)

#define PIN_STEP_ENABLE   11   // PumpEnable / ENA (active LOW)
#define PIN_STEP_DIR       9   // Direction+ (HIGH vs LOW = two directions)
#define PIN_STEP_PULSE     8   // Pull(PWM) / PUL input
#define PIN_HEAT_SWITCH   10   // HeatSwitch (active LOW to turn heater ON)

// Stepper timing – tune these for your motor / microstep settings
#define MAX_STEP_DELAY_US   5000   // slowest: 5 ms between steps
#define MIN_STEP_DELAY_US    500   // fastest: 0.5 ms between steps
#define STEP_PULSE_WIDTH_US   10   // pulse high time

// Map 0–100% speed into a delay between steps
static uint32_t speed_percent_to_delay_us(float speed_percent) {
    if (speed_percent <= 0.0f)  return 0;       // 0% = no movement
    if (speed_percent > 100.0f) speed_percent = 100.0f;

    float p = speed_percent / 100.0f;
    float delay_f =
        (float)MAX_STEP_DELAY_US -
        p * (float)(MAX_STEP_DELAY_US - MIN_STEP_DELAY_US);

    if (delay_f < (float)MIN_STEP_DELAY_US) delay_f = (float)MIN_STEP_DELAY_US;
    return (uint32_t)delay_f;
}

// Set direction (true = "up", false = "down")
// If you find it reversed, just flip the bool here.
void set_stepper_direction(bool up) {
    gpio_put(PIN_STEP_DIR, up ? 1 : 0);
}

// Enable/disable TB6600
void set_stepper_enabled(bool enabled) {
    // TB6600 ENA input is  active LOW:
    gpio_put(PIN_STEP_ENABLE, enabled ? 0 : 1);
}

// Move a given number of steps at a given speed (0–100%) and direction
void stepper_move_steps(int32_t steps, float speed_percent, bool direction_up) {
    if (steps <= 0) return;

    uint32_t delay_us = speed_percent_to_delay_us(speed_percent);
    if (delay_us == 0) return;   // 0% speed → no motion

    set_stepper_direction(direction_up);
    set_stepper_enabled(true);

    for (int32_t i = 0; i < steps; ++i) {
        // one rising edge on PUL = one step
        gpio_put(PIN_STEP_PULSE, 1);
        sleep_us(STEP_PULSE_WIDTH_US);
        gpio_put(PIN_STEP_PULSE, 0);
        sleep_us(delay_us);
    }

    // You can leave it enabled if you want holding torque;
    // or disable to save power:
    // set_stepper_enabled(false);
}

// Heater control: "Pull HeatSwitch LOW to turn on heater"
void set_heater(bool on) {
    gpio_put(PIN_HEAT_SWITCH, on ? 0 : 1);  // active LOW
}

// ================= Pump H-Bridge (DRV8871) =================
// From your PCB pinout
#define PIN_PUMP_IN1  12   // MotorIn1
#define PIN_PUMP_IN2  13   // MotorIn2

enum PumpMode {
    PUMP_COAST = 0,   // IN1=0 IN2=0 (Hi-Z / sleep)
    PUMP_FORWARD,     // IN1=1 IN2=0
    PUMP_REVERSE,     // IN1=0 IN2=1
    PUMP_BRAKE        // IN1=1 IN2=1
};

void pump_set_mode(PumpMode mode) {
    switch (mode) {
        case PUMP_COAST:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
        case PUMP_FORWARD:
            gpio_put(PIN_PUMP_IN1, 1);
            gpio_put(PIN_PUMP_IN2, 0);
            break;
        case PUMP_REVERSE:
            gpio_put(PIN_PUMP_IN1, 0);
            gpio_put(PIN_PUMP_IN2, 1);
            break;
        case PUMP_BRAKE:
            gpio_put(PIN_PUMP_IN1, 1);
            gpio_put(PIN_PUMP_IN2, 1);
            break;
    }
}

// Simple boolean interface (ROS-friendly)
void pump_set_enabled(bool enabled, bool forward) {
    if (!enabled) {
        pump_set_mode(PUMP_COAST);
    } else {
        pump_set_mode(forward ? PUMP_FORWARD : PUMP_REVERSE);
    }
}

// ================= main =================

int main() {
    stdio_init_all();
    sleep_ms(2000);                 // let USB enumerate

    printf("Hello from Science PCB!\n");

    // ---- I2C init ----
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ---- GPIO init for stepper + heater ----
    gpio_init(PIN_STEP_ENABLE);
    gpio_init(PIN_STEP_DIR);
    gpio_init(PIN_STEP_PULSE);
    gpio_init(PIN_HEAT_SWITCH);

    gpio_set_dir(PIN_STEP_ENABLE, GPIO_OUT);
    gpio_set_dir(PIN_STEP_DIR, GPIO_OUT);
    gpio_set_dir(PIN_STEP_PULSE, GPIO_OUT);
    gpio_set_dir(PIN_HEAT_SWITCH, GPIO_OUT);

        // ---- GPIO init for pump H-bridge ----
gpio_init(PIN_PUMP_IN1);
gpio_init(PIN_PUMP_IN2);
gpio_set_dir(PIN_PUMP_IN1, GPIO_OUT);
gpio_set_dir(PIN_PUMP_IN2, GPIO_OUT);

// Default pump state
pump_set_mode(PUMP_COAST);


    // Idle states
    set_stepper_enabled(false);
    gpio_put(PIN_STEP_PULSE, 0);
    set_heater(false);

    // Optional: scan I2C bus at startup
    i2c_scan();

    // Simple demo loop:
    //  - Read and print temp/humidity every second
    //  - (Example calls for stepper & heater are commented; uncomment to test)

    bool dir_up = true;

    while (true) {
        float t = read_temperature();
        float h = read_humidity();
        printf("Temp: %.2f C, Hum: %.2f %%\n", t, h);

        // --- Example: move stepper a bit and toggle heater ---
        // set_heater(true);                         // heater ON
        // stepper_move_steps(800, 50.0f, dir_up);   // 800 steps at 50% speed
        // set_heater(false);                        // heater OFF
        // dir_up = !dir_up;                         // flip direction next time

        sleep_ms(1000);
    }


}
