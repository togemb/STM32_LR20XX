#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>

/* Интерфейс LoRa-радио для приложения. Не зависит от конкретного чипа:
 * другой радиомодуль = другой radio_xxx.c с теми же функциями. */

typedef enum {
    RADIO_OK = 0,
    RADIO_ERR_COMM,    /* нет ответа: BUSY завис, ошибка SPI */
    RADIO_ERR_CHIP,    /* ответил не тот чип */
    RADIO_ERR_PRAM,    /* патч прошивки не загрузился */
    RADIO_ERR_CLOCK,   /* не запустился TCXO */
    RADIO_ERR_CONFIG,  /* чип отклонил команду настройки или неверный аргумент */
    RADIO_ERR_BUSY     /* предыдущая передача ещё не завершена */
} radio_status_t;

/* Результат опроса передачи */
typedef enum {
    RADIO_TX_BUSY = 0, /* идёт передача, ждём */
    RADIO_TX_DONE,     /* пакет передан */
    RADIO_TX_TIMEOUT,  /* передача не завершилась за отведённое время */
    RADIO_TX_ERROR     /* чип отклонил SetTx, сработала защита усилителя или ошибка SPI */
} radio_tx_result_t;

/* Результат опроса приёма */
typedef enum {
    RADIO_RX_NONE = 0,       /* ничего нового, ждём */
    RADIO_RX_DONE,           /* принят пакет, данные и параметры заполнены */
    RADIO_RX_CRC_ERROR,      /* пакет принят с ошибкой CRC, данные отброшены */
    RADIO_RX_HEADER_ERROR,   /* повреждён заголовок LoRa */
    RADIO_RX_ERROR           /* приём остановлен: чип отклонил команду, таймаут или ошибка SPI */
} radio_rx_result_t;

/* Параметры принятого пакета */
typedef struct {
    uint8_t len;               /* число байт в буфере */
    int16_t rssi_half_dbm;     /* уровень сигнала, единицы 0,5 дБм (-171 = -85,5 дБм) */
    int8_t  snr_quarter_db;    /* отношение сигнал/шум, единицы 0,25 дБ */
    uint8_t rssi_overload;
} radio_rx_info_t;

radio_status_t radio_lora_init(void);
radio_status_t radio_lora_get_version(uint8_t *major, uint8_t *minor);

/* Настраивает приём (тракт HF, калибровка, прерывания), но приём не запускает.
 * Вызывать после radio_lora_init(), когда передача не идёт. */
radio_status_t radio_lora_rx_config(void);

/* Запускает передачу len байт (1...255) и сразу возвращается. Завершение узнаём через radio_lora_tx_poll(). */
/* Запускает непрерывный приём и сразу возвращается. Перед этим нужна radio_lora_rx_config(). */
radio_status_t    radio_lora_rx_start(void);

/* Опрос приёма. Если пакет принят (RADIO_RX_DONE), копирует до buf_size байт в buf и заполняет info.
 * Приём продолжается сам, повторно запускать его не нужно. После RADIO_RX_ERROR приём остановлен. */
radio_rx_result_t radio_lora_rx_poll(uint8_t *buf, uint8_t buf_size, radio_rx_info_t *info);

radio_status_t    radio_lora_send(const uint8_t *data, uint8_t len);
radio_tx_result_t radio_lora_tx_poll(void);

#endif /* RADIO_H */
