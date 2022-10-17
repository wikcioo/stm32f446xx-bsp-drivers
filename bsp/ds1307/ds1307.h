#ifndef __DS1307_H__
#define __DS1307_H__

#include "stm32f446xx.h"

/* DS1307 configuration structure */
typedef struct
{
    i2c_regdef_t  *i2cx;
    gpio_regdef_t *gpiox;
    uint8_t        i2c_speed;
    uint8_t        i2c_slave_addr;
    uint8_t        gpio_scl_pin;
    uint8_t        gpio_sda_pin;
    uint8_t        gpio_pupd;
} ds1307_config_t;

/* DS1307 date structure */
typedef struct
{
    uint8_t year;   /* 00 - 99 */
    uint8_t month;  /* 01 - 12 */
    uint8_t date;   /* 01 - 31 */
    uint8_t day;    /* 01 - 07 */
} ds1307_date_t;

/* DS1307 time structure */
typedef struct
{
    uint8_t time_format;    /* see @time_format for configurable parameters */
    uint8_t hours;          /* 1 - 12 + AM/PM or 00 -23 */
    uint8_t minutes;        /* 00 - 59 */
    uint8_t seconds;        /* 00 - 59 */
} ds1307_time_t;

/* DS1307 public driver API */
uint8_t ds1307_init             (ds1307_config_t *config);
void    ds1307_set_current_time (ds1307_time_t *time);
void    ds1307_set_current_date (ds1307_date_t *date);
void    ds1307_get_current_time (ds1307_time_t *time);
void    ds1307_get_current_date (ds1307_date_t *date);

/* @time_format configuration */
#define DS1307_TIME_FORMAT_12HRS_AM 0
#define DS1307_TIME_FORMAT_12HRS_PM 1
#define DS1307_TIME_FORMAT_24HRS    2

/* DS1307 registers */
#define DS1307_REG_SEC      0x00
#define DS1307_REG_MIN      0x01
#define DS1307_REG_HRS      0x02
#define DS1307_REG_DAY      0x03
#define DS1307_REG_DATE     0x04
#define DS1307_REG_MONTH    0x05
#define DS1307_REG_YEAR     0x06

/* Other DS1307 macros */
#define DS1307_INIT_SUCCESS 0
#define DS1307_INIT_FAILURE 1

#define DS1307_SUNDAY       1
#define DS1307_MONDAY       2
#define DS1307_TUESDAY      3
#define DS1307_WEDNESDAY    4
#define DS1307_THURSDAY     5
#define DS1307_FRIDAY       6
#define DS1307_SATURDAY     7

#endif
