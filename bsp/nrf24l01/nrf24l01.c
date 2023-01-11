#include "nrf24l01.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

static spi_handle_t nrf24l01_spi_handle;

static gpio_regdef_t *gpio_chip_enable_port;
static uint8_t        gpio_chip_enable_pin;
static gpio_regdef_t *gpio_chip_select_port;
static uint8_t        gpio_chip_select_pin;

static void nrf24l01_spi_init           (nrf24l01_spi_config_t *config);
static void nrf24l01_single_byte_write  (uint8_t reg, uint8_t *tx_buffer);
static void nrf24l01_multi_byte_write   (uint8_t reg, uint8_t *tx_buffer, uint8_t length);
static void nrf24l01_single_byte_read   (uint8_t reg, uint8_t *rx_buffer);
static void nrf24l01_multi_byte_read    (uint8_t reg, uint8_t *rx_buffer, uint8_t length);

static void nrf24l01_chip_enable(void);
static void nrf24l01_chip_disable(void);
static void nrf24l01_chip_select(void);
static void nrf24l01_chip_unselect(void);

void nrf24l01_init(nrf24l01_handle_t *nrf24l01)
{
    gpio_chip_enable_port = nrf24l01->dev_config.gpiox_ce;
    gpio_chip_enable_pin  = nrf24l01->dev_config.gpio_ce_pin;
    gpio_chip_select_port = nrf24l01->spi_config.gpiox;
    gpio_chip_select_pin  = nrf24l01->spi_config.gpio_nss_pin;
    nrf24l01_spi_init(&nrf24l01->spi_config);

    nrf24l01_chip_disable();

    uint8_t data;

    data = 0;
    nrf24l01_single_byte_write(NRF24L01_REG_CONFIG, &data);
    nrf24l01_single_byte_write(NRF24L01_REG_EN_AA, &data);
    nrf24l01_single_byte_write(NRF24L01_REG_EN_RXADDR, &data);

    data = 0x3;
    nrf24l01_single_byte_write(NRF24L01_REG_SETUP_AW, &data);

    data = 0;
    nrf24l01_single_byte_write(NRF24L01_REG_SETUP_RETR, &data);
    nrf24l01_single_byte_write(NRF24L01_REG_RF_CH, &data);

    data = 0x0E;
    nrf24l01_single_byte_write(NRF24L01_REG_RF_SETUP, &data);

    nrf24l01_chip_enable();
}

static void nrf24l01_spi_init(nrf24l01_spi_config_t *config)
{
    gpio_handle_t spi_gpio = {0};
    spi_gpio.gpiox               = config->gpiox;
    spi_gpio.config.pin_mode     = GPIO_MODE_ALT_FUNC;
    spi_gpio.config.pin_alt_func = GPIO_ALT_FUNC_5;
    spi_gpio.config.pin_pupd     = GPIO_NO_PUPD;
    spi_gpio.config.pin_speed    = GPIO_SPEED_HIGH;

    spi_gpio.config.pin_number   = config->gpio_nss_pin;
    gpio_init(&spi_gpio);

    spi_gpio.config.pin_number   = config->gpio_sck_pin;
    gpio_init(&spi_gpio);

    spi_gpio.config.pin_number   = config->gpio_miso_pin;
    gpio_init(&spi_gpio);

    spi_gpio.config.pin_number   = config->gpio_mosi_pin;
    gpio_init(&spi_gpio);

    nrf24l01_spi_handle.spix           = config->spix;
    nrf24l01_spi_handle.config.mode    = SPI_MODE_MASTER;
    nrf24l01_spi_handle.config.comm    = SPI_COMM_FULL_DUPLEX;
    nrf24l01_spi_handle.config.clk_div = SPI_CLK_DIV_2;
    nrf24l01_spi_handle.config.cpol    = SPI_CPOL_IDLE_LOW;
    nrf24l01_spi_handle.config.cpha    = SPI_CPHA_FIRST_TRANSITION_CAPTURE;
    nrf24l01_spi_handle.config.ssm     = SPI_SSM_ENABLE;
    nrf24l01_spi_handle.config.dff     = SPI_DFF_8BITS;
    nrf24l01_spi_handle.config.ff      = SPI_FF_MSB_FIRST;
    spi_init(&nrf24l01_spi_handle);
}

static void nrf24l01_single_byte_write(uint8_t reg, uint8_t *tx_buffer)
{
    uint8_t buf[2];
    // in order to write to the register, the 5th bit must be set. See docs p.46
    buf[0] = reg | (1 << 5);
    buf[1] = *tx_buffer;

    nrf24l01_chip_select();

    spi_transmit(&nrf24l01_spi_handle, buf, 2);

    nrf24l01_chip_unselect();
}

static void nrf24l01_multi_byte_write(uint8_t reg, uint8_t *tx_buffer, uint8_t length)
{
    uint8_t buf[1];
    // in order to write to the register, the 5th bit must be set. See docs p.46
    buf[0] = reg | (1 << 5);

    nrf24l01_chip_select();

    spi_transmit(&nrf24l01_spi_handle, buf, 1);
    spi_transmit(&nrf24l01_spi_handle, tx_buffer, length);

    nrf24l01_chip_unselect();
}

static void nrf24l01_single_byte_read(uint8_t reg, uint8_t *rx_buffer)
{
    nrf24l01_chip_select();

    spi_transmit(&nrf24l01_spi_handle, &reg, 1);
    spi_receive(&nrf24l01_spi_handle, rx_buffer, 1);

    nrf24l01_chip_unselect();
}

static void nrf24l01_multi_byte_read(uint8_t reg, uint8_t *rx_buffer, uint8_t length)
{
    nrf24l01_chip_select();

    spi_transmit(&nrf24l01_spi_handle, &reg, 1);
    spi_receive(&nrf24l01_spi_handle, rx_buffer, length);

    nrf24l01_chip_unselect();
}

static void nrf24l01_chip_enable(void)
{
    gpio_write_pin(gpio_chip_enable_port, gpio_chip_enable_pin, GPIO_PIN_HIGH);
}

static void nrf24l01_chip_disable(void)
{
    gpio_write_pin(gpio_chip_enable_port, gpio_chip_enable_pin, GPIO_PIN_LOW);
}

static void nrf24l01_chip_select(void)
{
    gpio_write_pin(gpio_chip_select_port, gpio_chip_select_pin, GPIO_PIN_LOW);
}

static void nrf24l01_chip_unselect(void)
{
    gpio_write_pin(gpio_chip_select_port, gpio_chip_select_pin, GPIO_PIN_HIGH);
}

static void nrf24l01_send_cmd(uint8_t cmd)
{
    nrf24l01_chip_select();
    
    spi_transmit(&nrf24l01_spi_handle, &cmd, 1);

    nrf24l01_chip_unselect();
}
