/**
 * @file    lr2021.h
 * @brief   Слой радиочипа LR2021: инициализация, FSK-конфигурация, передача, приём.
 *
 * Правила слоя:
 *  - ничего не печатает, только возвращает коды и данные;
 *  - не зависит от HAL микроконтроллера (только bsp_* из board.h и драйвер Semtech);
 *  - не опрашивает ногу IRQ: вызывающий слой (board) смотрит на неё сам и при
 *    событии вызывает lr2021_irq_process();
 *  - параметры радио задаются в apps_configuration.h.
 */

#ifndef LR2021_H
#define LR2021_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Коды возврата. 0 = успех, отрицательные = ошибка. */
typedef enum {
    LR2021_OK         =  0,
    LR2021_ERR_PARAM  = -1,  /**< неверный аргумент                                     */
    LR2021_ERR_CONFIG = -2,  /**< параметры из apps_configuration.h недопустимы для чипа */
    LR2021_ERR_IO     = -3,  /**< ошибка команды: SPI, таймаут BUSY или отказ чипа        */
    LR2021_ERR_BUFFER = -4   /**< буфер мал для принятого пакета                          */
} lr2021_status_t;

/** События, возвращаемые lr2021_irq_process() (битовая маска). */
#define LR2021_EVT_TX_DONE     ( 1u << 0 )  /**< пакет передан                                       */
#define LR2021_EVT_RX_DONE     ( 1u << 1 )  /**< принят корректный пакет, можно вызвать read_packet   */
#define LR2021_EVT_CRC_ERROR   ( 1u << 2 )  /**< пакет принят с ошибкой CRC (отброшен, FIFO очищен)   */
#define LR2021_EVT_LEN_ERROR   ( 1u << 3 )  /**< длина пакета больше FSK_MAX_PAYLOAD_BYTES (отброшен)  */
#define LR2021_EVT_TIMEOUT     ( 1u << 4 )  /**< таймаут операции TX/RX                               */
#define LR2021_EVT_CHIP_ERROR  ( 1u << 5 )  /**< ошибка чипа, подробности: lr2021_get_errors()         */

/** Статистика приёма FSK из чипа. Счётчики 16-битные и переполняются. */
typedef struct {
    uint16_t received;     /**< принятые пакеты            */
    uint16_t crc_errors;   /**< пакеты с ошибкой CRC       */
    uint16_t len_errors;   /**< пакеты с ошибкой длины     */
    uint16_t sync_ok;      /**< успешные детекции синхрослова */
    uint16_t sync_fail;    /**< неудачные детекции синхрослова */
} lr2021_rx_stats_t;

/**
 * @brief Сброс, включение модуля, загрузка PRAM, настройка питания, тактирования,
 *        DIO (IRQ и RF-ключи), калибровка front-end. После неё чип в Standby RC.
 */
lr2021_status_t lr2021_init(void);

/** @brief Версия прошивки чипа. */
lr2021_status_t lr2021_get_version(uint8_t *major, uint8_t *minor);

/**
 * @brief Настройка радио под FSK по apps_configuration.h: тип пакета, частота, PA,
 *        модуляция, синхрослово, CRC, whitening, формат пакета. Вызывать после lr2021_init().
 *        Возвращает LR2021_ERR_CONFIG, если полоса приёма не найдена.
 */
lr2021_status_t lr2021_fsk_config(void);

/**
 * @brief Запуск передачи пакета (не блокирует). Завершение: LR2021_EVT_TX_DONE.
 *        Из режима приёма переводит чип в Standby, после передачи чип остаётся в Standby.
 * @param data  полезная нагрузка
 * @param len   1 .. FSK_MAX_PAYLOAD_BYTES
 */
lr2021_status_t lr2021_tx(const uint8_t *data, uint16_t len);

/** @brief Запуск непрерывного приёма (не блокирует). Чип остаётся в приёме после каждого пакета. */
lr2021_status_t lr2021_rx_start(void);

/** @brief Перевод в Standby RC (останавливает приём). */
lr2021_status_t lr2021_standby(void);

/**
 * @brief Прочитать и сбросить флаги IRQ чипа, преобразовать в LR2021_EVT_*.
 *        Для CRC_ERROR и LEN_ERROR очищает RX FIFO. Вызывать, когда board увидел IRQ.
 * @param events  [out] маска LR2021_EVT_* (0, если ничего нет)
 */
lr2021_status_t lr2021_irq_process(uint32_t *events);

/**
 * @brief Прочитать принятый пакет из RX FIFO (после LR2021_EVT_RX_DONE).
 * @param buf      буфер под нагрузку
 * @param buf_len  размер буфера
 * @param len      [out] длина пакета (при LR2021_ERR_BUFFER тоже заполняется, пакет отбрасывается)
 */
lr2021_status_t lr2021_read_packet(uint8_t *buf, uint16_t buf_len, uint16_t *len);

/** @brief RSSI последнего принятого пакета, дБм (после LR2021_EVT_RX_DONE). */
lr2021_status_t lr2021_get_packet_rssi(int16_t *rssi_dbm);

/** @brief Статистика приёма из чипа. */
lr2021_status_t lr2021_get_rx_stats(lr2021_rx_stats_t *stats);

/** @brief Сбросить статистику приёма в чипе. */
lr2021_status_t lr2021_reset_rx_stats(void);

/** @brief Прочитать и сбросить флаги ошибок чипа (при LR2021_EVT_CHIP_ERROR). */
lr2021_status_t lr2021_get_errors(uint16_t *errors);

#ifdef __cplusplus
}
#endif

#endif /* LR2021_H */
