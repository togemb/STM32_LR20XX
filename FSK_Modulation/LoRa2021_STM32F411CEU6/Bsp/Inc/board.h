#ifndef __BOARD_H
#define __BOARD_H

#include "main.h"

/* События для bsp_radio_poll() */
#define BSP_RADIO_EVT_TX_DONE     (1u << 0)  /* пакет передан                       */
#define BSP_RADIO_EVT_RX_DONE     (1u << 1)  /* принят корректный пакет             */
#define BSP_RADIO_EVT_CRC_ERROR   (1u << 2)  /* пакет с ошибкой CRC, отброшен       */
#define BSP_RADIO_EVT_LEN_ERROR   (1u << 3)  /* пакет длиннее допустимого, отброшен */
#define BSP_RADIO_EVT_TIMEOUT     (1u << 4)  /* таймаут TX/RX                       */
#define BSP_RADIO_EVT_CHIP_ERROR  (1u << 5)  /* ошибка чипа, см. bsp_radio_get_errors */

/* MCU =======================================================================*/
/* System --------------------------------------------------------------------*/
void     bsp_init(void);                       // запускает TIM2, вызывать после MX_*_Init()
uint32_t bsp_get_ms(void);
uint32_t bsp_get_us(void);                     // TIM2, шаг 1 мкс, переполнение через ~71 мин
void     bsp_delay_ms(uint32_t ms);

/* GPIO ----------------------------------------------------------------------*/
void     bsp_radio_nss_set(uint8_t level);
void     bsp_radio_reset_set(uint8_t level);   // 0 = сброс активен
void     bsp_radio_ce_set(uint8_t level);
uint8_t  bsp_radio_busy_read(void);
uint8_t  bsp_radio_irq_read(void);
uint8_t  bsp_radio_irq_take(void);             // был ли фронт на IRQ; флаг сбрасывается

/* SPI -----------------------------------------------------------------------*/
int      bsp_spi_write(const uint8_t *tx, uint16_t len);
int      bsp_spi_read(uint8_t *rx, uint16_t len);
int      bsp_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);  // tx или rx могут быть NULL

/* LOG -----------------------------------------------------------------------*/
int      bsp_uart_write(const uint8_t *data, uint16_t len);
int      __io_putchar(int ch);                 // для printf, имя задано newlib; объявлять в заголовке не обязательно


/* LR20XX ====================================================================*/
int bsp_radio_init(void);                      /* сброс, init чипа, конфигурация FSK */
uint32_t bsp_radio_get_bitrate(void);   /* битрейт в эфире, бит/с, для лога */
int bsp_radio_get_version(uint8_t *major, uint8_t *minor);
int bsp_radio_tx(const uint8_t *data, uint16_t len);   /* 1..255 байт, не блокирует */
int bsp_radio_rx_start(void);                  /* непрерывный приём, не блокирует   */
int bsp_radio_standby(void);
int bsp_radio_poll(uint32_t *events);          /* вызывать из основного цикла       */
int bsp_radio_read_packet(uint8_t *buf, uint16_t buf_len, uint16_t *len);
int bsp_radio_get_rssi(int16_t *rssi_dbm);
int bsp_radio_get_rx_stats(uint16_t *received, uint16_t *crc_errors, uint16_t *len_errors);
int bsp_radio_reset_rx_stats(void);
int bsp_radio_get_errors(uint16_t *errors);
#endif /*__BOARD_H*/