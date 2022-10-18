#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f446xx.h"

/* BMP280 pin configuration structure */
typedef struct
{
    i2c_regdef_t  *i2cx;
    gpio_regdef_t *gpiox;
    uint8_t        i2c_speed;
    uint8_t        i2c_slave_addr;
    uint8_t        gpio_scl_pin;
    uint8_t        gpio_sda_pin;
    uint8_t        gpio_pupd;
} bmp280_pin_config_t;

/* BMP280 device configuration structure */
typedef struct
{
    uint8_t osrs_t;     /* see @osrs configuration */
    uint8_t osrs_p;     /* see @osrs configuration */
    uint8_t mode;       /* see @mode configuration */
    uint8_t t_sb;       /* see @standby_time configuration */
    uint8_t filter;     /* see @IIR_filter_coefficient configuration */
} bmp280_dev_config_t;

/* BMP280 handle structure */
typedef struct
{
    bmp280_pin_config_t pin_config;
    bmp280_dev_config_t dev_config;

    float temperature;
    float pressure;
} bmp280_handle_t;

/* BMP280 public driver API */
uint8_t bmp280_init    (bmp280_handle_t *bmp280);
void    bmp280_measure (bmp280_handle_t *bmp280);
void    bmp280_wakeup  (void);

/* @osrs configuration */
#define BMP280_OSRS_OFF     0x00
#define BMP280_OSRS_1       0x01
#define BMP280_OSRS_2       0x02
#define BMP280_OSRS_4       0x03
#define BMP280_OSRS_8       0x04
#define BMP280_OSRS_16      0x05

/* @mode configuration */
#define BMP280_MODE_SLEEP   0x00
#define BMP280_MODE_FORCED  0x01
#define BMP280_MODE_NORMAL  0x02

/* @standby_time configuration */
#define BMP280_T_SB_0_5     0x00
#define BMP280_T_SB_62_5    0x01
#define BMP280_T_SB_125     0x02
#define BMP280_T_SB_250     0x03
#define BMP280_T_SB_500     0x04
#define BMP280_T_SB_1000    0x05
#define BMP280_T_SB_2000    0x06
#define BMP280_T_SB_4000    0x07

/* @IIR_filter_coefficient configuration */
#define BMP280_IIR_OFF      0x00
#define BMP280_IIR_2        0x01
#define BMP280_IIR_4        0x02
#define BMP280_IIR_8        0x03
#define BMP280_IIR_16       0x04

/* BMP280 registers */
#define BMP280_REG_CALIB00      0x88
#define BMP280_REG_ID           0xD0
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_TEMP_MSB     0xFA

/* Other BMP280 macros */
#define BMP280_CHIP_ID      0x58
#define BMP280_INIT_SUCCESS 0
#define BMP280_INIT_FAILURE 1

#endif
