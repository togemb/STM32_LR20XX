#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include <stdbool.h>

/* Плата: всё, что зависит от МК и разводки. Единственный слой, знающий про CubeMX. */

void     board_init(void);               /* запуск TIM2 как счётчика микросекунд */

void     board_delay_us(uint32_t us);
void     board_delay_ms(uint32_t ms);
uint32_t board_get_ms(void);

/* Линии радиомодуля */
void board_radio_ce(bool level);
void board_radio_nrst(bool level);
void board_radio_nss(bool level);
bool board_radio_busy(void);
bool board_radio_irq(void);                 /* текущий уровень на выводе IRQ модуля */
bool board_radio_irq_take(void);            /* был ли фронт на IRQ с прошлого вызова (флаг сбрасывается) */

/* SPI-обмен. Возвращают false при ошибке или таймауте. */
bool board_radio_spi_write(const uint8_t *data, uint16_t len);
bool board_radio_spi_read(uint8_t *data, uint16_t len);   /* на MOSI уходят нули */

#endif /* BOARD_H */
