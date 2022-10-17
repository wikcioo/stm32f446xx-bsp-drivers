#include "ds1307.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_i2c.h"

static i2c_handle_t ds1307_i2c_handle;
static uint8_t ds1307_slave_addr;

static void    ds1307_i2c_init   (ds1307_config_t *config);
static void    ds1307_write      (uint8_t reg, uint8_t value);
static uint8_t ds1307_read       (uint8_t reg);
static uint8_t ds1307_bin_to_bcd (uint8_t value);
static uint8_t ds1307_bcd_to_bin (uint8_t value);

uint8_t ds1307_init(ds1307_config_t *config)
{
    ds1307_slave_addr = config->i2c_slave_addr;
    ds1307_i2c_init(config);

    /* Enable clock oscillator */
    ds1307_write(DS1307_REG_SEC, 0x00);

    /* Returns SUCCESS when clock oscillator has been activated */
    return ((ds1307_read(DS1307_REG_SEC) >> 7) & 0x1) ? DS1307_INIT_FAILURE : DS1307_INIT_SUCCESS;
}

void ds1307_set_current_time(ds1307_time_t *time)
{
    /* Set seconds (keeping CH = 0) */
    ds1307_write(DS1307_REG_SEC, ds1307_bin_to_bcd(time->seconds) & ~(1 << 7));

    /* Set minutes */
    ds1307_write(DS1307_REG_MIN, ds1307_bin_to_bcd(time->minutes));

    /* Set hours based on time format */
    if (time->time_format == DS1307_TIME_FORMAT_24HRS) {
        /* Clear 6th bit of DS1307_REG_HRS to keep 24 hours format */
        ds1307_write(DS1307_REG_HRS, ds1307_bin_to_bcd(time->hours) & ~(1 << 6));
    }
    else {
        if (time->time_format == DS1307_TIME_FORMAT_12HRS_AM) {
            /* Set 6th bit of DS1307_REG_HRS to keep 12 hours format */
            /* Clear 5th bit of DS1307_REG_HRS to keep AM format */
            ds1307_write(DS1307_REG_HRS, (ds1307_bin_to_bcd(time->hours) | (1 << 6)) & ~(1 << 5));
        }
        else {
            /* Set 6th bit of DS1307_REG_HRS to keep 12 hours format */
            /* Set 5th bit of DS1307_REG_HRS to keep PM format */
            ds1307_write(DS1307_REG_HRS, ds1307_bin_to_bcd(time->hours) | (1 << 6) | (1 << 5));
        }
    }
}

void ds1307_set_current_date(ds1307_date_t *date)
{
    ds1307_write(DS1307_REG_YEAR,  ds1307_bin_to_bcd(date->year) );
    ds1307_write(DS1307_REG_MONTH, ds1307_bin_to_bcd(date->month));
    ds1307_write(DS1307_REG_DATE,  ds1307_bin_to_bcd(date->date) );
    ds1307_write(DS1307_REG_DAY,   ds1307_bin_to_bcd(date->day)  );
}

void ds1307_get_current_time(ds1307_time_t *time)
{
    time->seconds = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_SEC) & 0x7F);
    time->minutes = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_MIN));
    time->hours   = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_HRS));

    if ((time->hours & (1 << 6)) == 0) {
        /* 24 hours format (last 6 bits are valid) */
        time->hours &= 0x3F;
        time->time_format = DS1307_TIME_FORMAT_24HRS;
    } else {
        /* 12 hours format (last 5 bits are valid) */
        time->hours &= 0x1F;
        if ((time->hours & (1 << 5)) == 0) {
            /* AM */
            time->time_format = DS1307_TIME_FORMAT_12HRS_AM;
        } else {
            /* PM */
            time->time_format = DS1307_TIME_FORMAT_12HRS_PM;
        }
    }
}

void ds1307_get_current_date(ds1307_date_t *date)
{
    date->year  = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_YEAR) );
    date->month = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_MONTH));
    date->date  = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_DATE) );
    date->day   = ds1307_bcd_to_bin(ds1307_read(DS1307_REG_DAY)  );
}

static void ds1307_i2c_init(ds1307_config_t *config)
{
    gpio_handle_t i2c_gpio_pin = {0};

    i2c_gpio_pin.gpiox                  = config->gpiox;
    i2c_gpio_pin.config.pin_mode        = GPIO_MODE_ALT_FUNC;
    i2c_gpio_pin.config.pin_alt_func    = GPIO_ALT_FUNC_4;
    i2c_gpio_pin.config.pin_speed       = GPIO_SPEED_FAST;
    i2c_gpio_pin.config.pin_pupd        = config->gpio_pupd;
    i2c_gpio_pin.config.pin_output_type = GPIO_OUTPUT_OPEN_DRAIN;

    i2c_gpio_pin.config.pin_number      = config->gpio_scl_pin;
    gpio_init(&i2c_gpio_pin);

    i2c_gpio_pin.config.pin_number      = config->gpio_sda_pin;
    gpio_init(&i2c_gpio_pin);

    ds1307_i2c_handle.i2cx               = config->i2cx;
    ds1307_i2c_handle.config.ack_control = I2C_ACK_ENABLE;
    ds1307_i2c_handle.config.clk_speed   = config->i2c_speed;;
    i2c_init(&ds1307_i2c_handle);
}

static void ds1307_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    i2c_master_transmit(&ds1307_i2c_handle, buf, 2, ds1307_slave_addr, I2C_STOP_BIT_DISABLE);
}

static uint8_t ds1307_read(uint8_t reg)
{
    uint8_t buf;
    i2c_master_transmit(&ds1307_i2c_handle, &reg, 1, ds1307_slave_addr, I2C_STOP_BIT_ENABLE);
    i2c_master_receive(&ds1307_i2c_handle, &buf, 1, ds1307_slave_addr, I2C_STOP_BIT_DISABLE);
    return buf;
}

static uint8_t ds1307_bin_to_bcd(uint8_t value)
{
    if (value >= 10) {
        uint8_t l = value / 10;
        uint8_t r = value % 10;
        return (l << 4) | r;
    }

    return value;
}

static uint8_t ds1307_bcd_to_bin(uint8_t value)
{
    uint8_t l = (value >> 4) & 0x0F;
    uint8_t r = value & 0x0F;
    return l * 10 + r;
}
