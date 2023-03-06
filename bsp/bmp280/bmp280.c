#include "bmp280.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_i2c.h"

#if !defined(BMP280_SUPPORT_64BIT) || !defined(BMP280_SUPPORT_32BIT)
    #define BMP280_SUPPORT_32BIT
#endif

static void     bmp280_i2c_init           (bmp280_pin_config_t *config);
static void     bmp280_single_byte_write  (uint8_t reg, uint8_t *tx_buffer);
static void     bmp280_multi_byte_write   (uint8_t reg, uint8_t *tx_buffer, uint8_t length);
static void     bmp280_single_byte_read   (uint8_t reg, uint8_t *rx_buffer);
static void     bmp280_multi_byte_read    (uint8_t reg, uint8_t *rx_buffer, uint8_t length);
static void     bmp280_read_trim_params   (void);
static void     bmp280_read_raw_data      (void);
static uint8_t  bmp280_is_valid_chip      (void);

static int32_t  bmp280_compensate_t_int32 (int32_t adc_T);
static uint32_t bmp280_compensate_p_int64 (int32_t adc_P);
static uint32_t bmp280_compensate_p_int32 (int32_t adc_P);

static i2c_handle_t bmp280_i2c_handle;
static uint8_t bmp280_slave_addr;

static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;

static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

static int32_t t_raw, p_raw;

uint8_t bmp280_init(bmp280_handle_t *bmp280)
{
    bmp280_i2c_init(&bmp280->pin_config);

    /* Validate the device by checking the chip_id */
    if (!bmp280_is_valid_chip())
        return BMP280_INIT_FAILURE;

    bmp280_read_trim_params();

    uint8_t data_to_write, data_check;

    /* Reset the device */
    data_to_write = 0xB6;
    bmp280_single_byte_write(BMP280_REG_RESET, &data_to_write);

    /* Set oversampling for temperature and pressure as well as the mode */
    data_to_write = ((bmp280->dev_config.osrs_t & 0x7) << 5) |
                    ((bmp280->dev_config.osrs_p & 0x7) << 2) |
                    (bmp280->dev_config.mode & 0x3);
    bmp280_single_byte_write(BMP280_REG_CTRL_MEAS, &data_to_write);
    bmp280_single_byte_read(BMP280_REG_CTRL_MEAS, &data_check);
    if (data_check != data_to_write)
        return BMP280_INIT_FAILURE;

    /* Set standby time and IIR filtering */
    data_to_write = ((bmp280->dev_config.t_sb & 0x7) << 5) | ((bmp280->dev_config.filter & 0x7) << 2);
    bmp280_single_byte_write(BMP280_REG_CONFIG, &data_to_write);
    bmp280_single_byte_read(BMP280_REG_CONFIG, &data_check);
    if (data_check != data_to_write)
        return BMP280_INIT_FAILURE;

    return BMP280_INIT_SUCCESS;
}

void bmp280_measure(bmp280_handle_t *bmp280)
{
    if (bmp280_is_valid_chip())
    {
        if (t_raw == 0x800000)
            bmp280->temperature = 0; /* Temperature measurements disabled */
        else
            bmp280->temperature = bmp280_compensate_t_int32(t_raw) / 100.0f;

        if (p_raw == 0x800000)
            bmp280->pressure = 0; /* Pressure measurements disabled */
        else
        {
#if defined(BMP280_SUPPORT_64BIT)
            bmp280->pressure = bmp280_compensate_p_int64(p_raw) / 256.0f;
#elif defined(BMP280_SUPPORT_32BIT)
            bmp280->pressure = bmp280_compensate_p_int32(p_raw);
#endif
        }
    }
    else
    {
        bmp280->temperature = 0;
        bmp280->pressure = 0;
    }
}

void bmp280_wakeup(void)
{
    uint8_t ctrl_measurements;
    bmp280_single_byte_read(BMP280_REG_CTRL_MEAS, &ctrl_measurements);

    /* Clear the last two bits */
    ctrl_measurements &= 0x3;
    /* Set the last two bits to be a forced mode */
    ctrl_measurements |= BMP280_MODE_FORCED;

    bmp280_single_byte_write(BMP280_REG_CTRL_MEAS, &ctrl_measurements);
}

static void bmp280_i2c_init(bmp280_pin_config_t *config)
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

    bmp280_i2c_handle.i2cx               = config->i2cx;
    bmp280_i2c_handle.config.ack_control = I2C_ACK_ENABLE;
    bmp280_i2c_handle.config.clk_speed   = config->i2c_speed;;
    i2c_init(&bmp280_i2c_handle);
}

static void bmp280_single_byte_write(uint8_t reg, uint8_t *tx_buffer)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = *tx_buffer;
    i2c_master_transmit(&bmp280_i2c_handle, buf, 2, bmp280_slave_addr, I2C_STOP_BIT_ENABLE);
}

static void bmp280_multi_byte_write(uint8_t reg, uint8_t *tx_buffer, uint8_t length)
{
    uint8_t buf[256 * 2];
    for (uint8_t i = 0; i < length; i += 2) {
        buf[i]   = reg++;
        buf[i+1] = *tx_buffer++;
    }

    i2c_master_transmit(&bmp280_i2c_handle, buf, length * 2, bmp280_slave_addr, I2C_STOP_BIT_ENABLE);
}

static void bmp280_single_byte_read(uint8_t reg, uint8_t *rx_buffer)
{
    i2c_master_transmit(&bmp280_i2c_handle, &reg, 1, bmp280_slave_addr, I2C_STOP_BIT_DISABLE);
    i2c_master_receive(&bmp280_i2c_handle, rx_buffer, 1, bmp280_slave_addr, I2C_STOP_BIT_ENABLE);
}

static void bmp280_multi_byte_read(uint8_t reg, uint8_t *rx_buffer, uint8_t length)
{
    i2c_master_transmit(&bmp280_i2c_handle, &reg, 1, bmp280_slave_addr, I2C_STOP_BIT_DISABLE);
    i2c_master_receive(&bmp280_i2c_handle, rx_buffer, length, bmp280_slave_addr, I2C_STOP_BIT_ENABLE);
}

static void bmp280_read_trim_params(void)
{
    uint8_t data[24];
    bmp280_multi_byte_read(BMP280_REG_CALIB00, data, 24);

    /* Populate dig_* as per the datasheet p.21 */
    dig_T1 = (data[1]  << 8) | data[0];
    dig_T2 = (data[3]  << 8) | data[2];
    dig_T3 = (data[5]  << 8) | data[4];
    dig_P1 = (data[7]  << 8) | data[6];
    dig_P2 = (data[9]  << 8) | data[8];
    dig_P3 = (data[11] << 8) | data[10];
    dig_P4 = (data[13] << 8) | data[12];
    dig_P5 = (data[15] << 8) | data[14];
    dig_P6 = (data[17] << 8) | data[16];
    dig_P7 = (data[19] << 8) | data[18];
    dig_P8 = (data[21] << 8) | data[20];
    dig_P9 = (data[23] << 8) | data[22];
}

static void bmp280_read_raw_data(void)
{
    uint8_t raw_data[6];
    bmp280_multi_byte_read(BMP280_REG_PRESS_MSB, raw_data, 6);

    /* Populate raw values as per the datasheet p.26 */
    p_raw = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
    t_raw = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
}

static uint8_t bmp280_is_valid_chip(void)
{
    uint8_t chip_id;
    bmp280_single_byte_read(BMP280_REG_ID, &chip_id);

    return chip_id == BMP280_CHIP_ID;
}

/* Compensation calculations found in datasheet p.22 */

/*
 * Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
 * t_fine carries fine temperature as global value
 */
int32_t t_fine;
int32_t bmp280_compensate_t_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}


#if defined(BMP280_SUPPORT_64BIT)
/*
 * Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
 * Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
 */
uint32_t bmp280_compensate_p_int64(int32_t adc_P)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

#elif defined(BMP280_SUPPORT_32BIT)
/*
 * Returns pressure in Pa as unsigned 32 bit integer.
 * Output value of “96386” equals 96386 Pa = 963.86 hPa
 */
uint32_t bmp280_compensate_p_int32(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) *var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}
#endif
