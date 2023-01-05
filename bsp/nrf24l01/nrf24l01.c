#include "nrf24l01.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

static spi_handle_t nrf24l01_spi_handle;

static void nrf24l01_spi_init(nrf24l01_spi_config_t *config);

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
