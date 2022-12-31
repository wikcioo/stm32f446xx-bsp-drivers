#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "stm32f446xx.h"

/* NRF24L01 configuration structure */
typedef struct
{
    spi_regdef_t  *spix;
    gpio_regdef_t *gpiox;
    uint8_t gpio_sck_pin;
    uint8_t gpio_miso_pin;
    uint8_t gpio_mosi_pin;
    uint8_t gpio_nss_pin;
} nrf24l01_spi_config_t;

/* NRF24L01 device configuration structure */
typedef struct
{
    gpio_regdef_t *gpiox_ce;
    uint8_t gpio_ce_pin;
    uint8_t mode;
    uint8_t addr;
} nrf24l01_dev_config_t;

/* NRF24L01 handle structure */
typedef struct
{
    nrf24l01_spi_config_t spi_config;
    nrf24l01_dev_config_t dev_config;

    uint8_t *tx_buffer;
    uint8_t  tx_buffer_length;
    uint8_t *rx_buffer;
    uint8_t  rx_buffer_length;
} nrf24l01_handle_t;

/* NRF24L01 public driver API */
void    nrf24l01_init               (nrf24l01_handle_t *nrf24l01);

void    nrf24l01_set_tx_mode        (uint8_t *address, uint8_t channel);
uint8_t nrf24l01_transmit           (uint8_t *data);

void    nrf24l01_set_rx_mode        (uint8_t *address, uint8_t channel);
void    nrf24l01_receive            (uint8_t *data);
uint8_t nrf24l01_is_data_available  (int pipenum);

void    nrf24l01_read_all           (uint8_t *data);

/* NRF24L01 registers */
#define NRF24L01_REG_CONFIG             0x00
#define NRF24L01_REG_EN_AA              0x01
#define NRF24L01_REG_EN_RXADDR          0x02
#define NRF24L01_REG_SETUP_AW           0x03
#define NRF24L01_REG_SETUP_RETR         0x04
#define NRF24L01_REG_RF_CH              0x05
#define NRF24L01_REG_RF_SETUP           0x06
#define NRF24L01_REG_STATUS             0x07
#define NRF24L01_REG_OBSERVE_TX         0x08
#define NRF24L01_REG_CD                 0x09
#define NRF24L01_REG_RX_ADDR_P0         0x0A
#define NRF24L01_REG_RX_ADDR_P1         0x0B
#define NRF24L01_REG_RX_ADDR_P2         0x0C
#define NRF24L01_REG_RX_ADDR_P3         0x0D
#define NRF24L01_REG_RX_ADDR_P4         0x0E
#define NRF24L01_REG_RX_ADDR_P5         0x0F
#define NRF24L01_REG_TX_ADDR            0x10
#define NRF24L01_REG_RX_PW_P0           0x11
#define NRF24L01_REG_RX_PW_P1           0x12
#define NRF24L01_REG_RX_PW_P2           0x13
#define NRF24L01_REG_RX_PW_P3           0x14
#define NRF24L01_REG_RX_PW_P4           0x15
#define NRF24L01_REG_RX_PW_P5           0x16
#define NRF24L01_REG_FIFO_STATUS        0x17
#define NRF24L01_REG_DYNPD	            0x1C
#define NRF24L01_REG_FEATURE	        0x1D

/* Commands */
#define NRF24L01_CMD_R_REGISTER         0x00
#define NRF24L01_CMD_W_REGISTER         0x20
#define NRF24L01_CMD_REGISTER_MASK      0x1F
#define NRF24L01_CMD_ACTIVATE           0x50
#define NRF24L01_CMD_R_RX_PL_WID        0x60
#define NRF24L01_CMD_R_RX_PAYLOAD       0x61
#define NRF24L01_CMD_W_TX_PAYLOAD       0xA0
#define NRF24L01_CMD_W_ACK_PAYLOAD      0xA8
#define NRF24L01_CMD_FLUSH_TX           0xE1
#define NRF24L01_CMD_FLUSH_RX           0xE2
#define NRF24L01_CMD_REUSE_TX_PL        0xE3
#define NRF24L01_CMD_NOP                0xFF

/* Other macros */
#define NRF24L01_SUCCESS 1
#define NRF24L01_FAILURE 0

#endif
