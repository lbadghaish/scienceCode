#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C on GP4 / GP5 (I2C0)
#define I2C_PORT i2c1
#define I2C_SDA  2     // GP4
#define I2C_SCL  3     // GP5

// SHT20 (if that’s your sensor)
#define SHT20_ADDR  0x40
#define TRIGGER_TEMP 0xF3
#define TRIGGER_HUM  0xF5

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

// ----------------- main -----------------
int main() {
    stdio_init_all();
    sleep_ms(2000);                 // let USB enumerate

    printf("Hello from Pico + I2C!\n");

    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz, nice and safe
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // 1) Scan once at startup
    i2c_scan();

    // 2) Repeatedly read sensor
    while (true) {
        float t = read_temperature();
        float h = read_humidity();
        printf("Temp: %.2f C, Hum: %.2f %%\n", t, h);
        sleep_ms(1000);
    }
}
