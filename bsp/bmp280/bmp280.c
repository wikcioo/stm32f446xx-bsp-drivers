#include "bmp280.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_i2c.h"

#if !defined(BMP280_SUPPORT_64BIT) || !defined(BMP280_SUPPORT_32BIT)
    #define BMP280_SUPPORT_32BIT
#endif

static i2c_handle_t bmp280_i2c_handle;
static uint8_t bmp280_slave_addr;

static void    bmp280_i2c_init         (bmp280_pin_config_t *config);
static void    bmp280_write            (uint8_t reg, uint8_t value);
static uint8_t bmp280_read             (uint8_t reg);
static void    bmp280_read_trim_params (void);

uint16_t dig_T1;
int16_t  dig_T2, dig_T3;

uint16_t dig_P1;
int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

uint8_t bmp280_init(bmp280_handle_t *bmp280)
{
}

void bmp280_measure(void)
{
}

void bmp280_wakeup(void)
{
}

static void bmp280_i2c_init(bmp280_pin_config_t *config)
{
}

static void bmp280_write(uint8_t reg, uint8_t value)
{
}

static uint8_t bmp280_read(uint8_t reg)
{
}

static void bmp280_read_trim_params(void)
{
}
