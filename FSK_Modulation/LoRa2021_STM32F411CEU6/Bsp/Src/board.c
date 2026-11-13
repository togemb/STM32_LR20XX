/**
 * @file    board.c
 * @brief   Слой платы (STM32F401RCT6): SPI, GPIO радиомодуля, время, UART-лог.
 *
 * Правила слоя:
 *  - файл не включает app.h и ничего не печатает, только возвращает коды;
 *  - все функции, видимые снаружи, начинаются с bsp_
 *    (исключение: __io_putchar, имя требует newlib/syscalls.c);
 *  - коды возврата int: 0 = успех, -1 = ошибка.
 *
 * Радио-обёртки bsp_radio_* скрывают от app тип чипа: app не знает ни про
 * lr2021.h, ни про драйвер Semtech. Коды возврата те же, что у lr2021:
 * 0 = успех, -1 = неверный аргумент, -2 = недопустимая конфигурация,
 * -3 = ошибка SPI/BUSY/чипа, -4 = буфер мал.
 */

#include <stddef.h>
#include <stdint.h>

#include "board.h"
#include "apps_configuration.h"
#include "lr2021.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* ------------------------------------------------------------------------- */
/* Настройки                                                                 */
/* ------------------------------------------------------------------------- */

/* Таймаут одной SPI-транзакции HAL, мс. Если board.h его уже задаёт, берётся оттуда. */
#ifndef BSP_SPI_TIMEOUT_MS
#define BSP_SPI_TIMEOUT_MS      10u
#endif

/* Таймаут передачи одного блока в UART, мс. */
#ifndef BSP_UART_TIMEOUT_MS
#define BSP_UART_TIMEOUT_MS     100u
#endif

/* Размер блока нулей для чтения по SPI (MOSI = 0x00 / NOP). */
#define BSP_SPI_ZERO_CHUNK      64u

/* ------------------------------------------------------------------------- */
/* Приватные данные                                                          */
/* ------------------------------------------------------------------------- */

/* Нули для тактирования SPI при чтении. HAL не пишет в tx-буфер, const безопасен. */
static const uint8_t s_spi_zeros[BSP_SPI_ZERO_CHUNK] = { 0 };

/* Флаг «на ноге IRQ был фронт». Ставится в прерывании, снимается в bsp_radio_irq_take(). */
static volatile uint8_t s_radio_irq_flag = 0;

static inline int prv_hal_rc(HAL_StatusTypeDef st)
{
    return (st == HAL_OK) ? 0 : -1;
}

/* ------------------------------------------------------------------------- */
/* Инициализация                                                             */
/* ------------------------------------------------------------------------- */

/**
 * @brief Запуск ресурсов платы, которые CubeMX только настраивает.
 *        Вызывать один раз после MX_*_Init(), до работы с радио.
 * @note  TIM2: 32 бита, 1 МГц (шаг 1 мкс), переполнение через ~71.6 минуты.
 */
void bsp_init(void)
{
    s_radio_irq_flag = 0;
    (void)HAL_TIM_Base_Start(&htim2);
}

/* ------------------------------------------------------------------------- */
/* Время                                                                     */
/* ------------------------------------------------------------------------- */

/** @brief Миллисекунды с запуска (SysTick). Разности считать как uint32_t. */
uint32_t bsp_get_ms(void)
{
    return HAL_GetTick();
}

/** @brief Микросекунды (TIM2). Разности считать как uint32_t, переполнение учтено. */
uint32_t bsp_get_us(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void bsp_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ------------------------------------------------------------------------- */
/* GPIO радиомодуля                                                          */
/* ------------------------------------------------------------------------- */

/** @param level 0 = low, не 0 = high */
void bsp_radio_nss_set(uint8_t level)
{
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @param level 0 = сброс активен (NRESET low), не 0 = работа */
void bsp_radio_reset_set(uint8_t level)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @param level 0 = модуль выключен, не 0 = включён (CE) */
void bsp_radio_ce_set(uint8_t level)
{
    HAL_GPIO_WritePin(LORA_CE_GPIO_Port, LORA_CE_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @return 1, если BUSY = high (чип занят), иначе 0 */
uint8_t bsp_radio_busy_read(void)
{
    return (HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/** @return текущий уровень ноги IRQ (1 = high) */
uint8_t bsp_radio_irq_read(void)
{
    return (HAL_GPIO_ReadPin(LORA_IRQ_GPIO_Port, LORA_IRQ_Pin) == GPIO_PIN_SET) ? 1u : 0u;
}

/**
 * @brief Был ли фронт на IRQ с прошлого вызова. Флаг сбрасывается.
 * @return 1 = было событие, 0 = нет
 */
uint8_t bsp_radio_irq_take(void)
{
    uint8_t v;

    __disable_irq();
    v = s_radio_irq_flag;
    s_radio_irq_flag = 0;
    __enable_irq();

    return v;
}

/** Переопределяет weak-функцию HAL. Вызывается из EXTI4_IRQHandler. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == LORA_IRQ_Pin) {
        s_radio_irq_flag = 1;
    }
}

/* ------------------------------------------------------------------------- */
/* SPI1 (CS управляется отдельно через bsp_radio_nss_set)                    */
/* ------------------------------------------------------------------------- */

/** @brief Только передача. */
int bsp_spi_write(const uint8_t *tx, uint16_t len)
{
    if (len == 0u) {
        return 0;
    }
    if (tx == NULL) {
        return -1;
    }
    return prv_hal_rc(HAL_SPI_Transmit(&hspi1, (uint8_t *)tx, len, BSP_SPI_TIMEOUT_MS));
}

/**
 * @brief Полнодуплексный обмен.
 * @param tx  данные для передачи или NULL (тогда выдаются нули)
 * @param rx  буфер приёма или NULL (тогда принятое отбрасывается)
 * @param len число байт
 */
int bsp_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (len == 0u) {
        return 0;
    }

    if (rx == NULL) {
        return bsp_spi_write(tx, len);
    }

    if (tx != NULL) {
        return prv_hal_rc(HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, len, BSP_SPI_TIMEOUT_MS));
    }

    /* tx == NULL: тактируем нулями кусками, длина не ограничена размером буфера нулей */
    while (len > 0u) {
        uint16_t n = (len > BSP_SPI_ZERO_CHUNK) ? (uint16_t)BSP_SPI_ZERO_CHUNK : len;

        if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)s_spi_zeros, rx, n, BSP_SPI_TIMEOUT_MS) != HAL_OK) {
            return -1;
        }
        rx  += n;
        len -= n;
    }
    return 0;
}

/** @brief Только приём. На MOSI выдаются нули (NOP), длина любая. */
int bsp_spi_read(uint8_t *rx, uint16_t len)
{
    return bsp_spi_transfer(NULL, rx, len);
}

/* ------------------------------------------------------------------------- */
/* Радио (поверх lr2021_*)                                                   */
/* ------------------------------------------------------------------------- */

/* События чипа -> события платы (app видит только BSP_RADIO_EVT_*) */
static uint32_t prv_map_events(uint32_t chip_events)
{
    uint32_t ev = 0u;

    if ((chip_events & LR2021_EVT_TX_DONE) != 0u)    { ev |= BSP_RADIO_EVT_TX_DONE; }
    if ((chip_events & LR2021_EVT_RX_DONE) != 0u)    { ev |= BSP_RADIO_EVT_RX_DONE; }
    if ((chip_events & LR2021_EVT_CRC_ERROR) != 0u)  { ev |= BSP_RADIO_EVT_CRC_ERROR; }
    if ((chip_events & LR2021_EVT_LEN_ERROR) != 0u)  { ev |= BSP_RADIO_EVT_LEN_ERROR; }
    if ((chip_events & LR2021_EVT_TIMEOUT) != 0u)    { ev |= BSP_RADIO_EVT_TIMEOUT; }
    if ((chip_events & LR2021_EVT_CHIP_ERROR) != 0u) { ev |= BSP_RADIO_EVT_CHIP_ERROR; }

    return ev;
}

/**
 * @brief Сброс и настройка радио: инициализация чипа и конфигурация FSK
 *        по apps_configuration.h. Вызывать после bsp_init().
 */
int bsp_radio_init(void)
{
    int rc = (int)lr2021_init();

    if (rc != 0) {
        return rc;
    }
    return (int)lr2021_fsk_config();
}

/** @brief Битрейт в эфире, бит/с, из apps_configuration.h (для лога в app). */
uint32_t bsp_radio_get_bitrate(void)
{
    return (uint32_t)FSK_BITRATE_BPS;
}

/** @brief Версия прошивки чипа (для лога в app). */
int bsp_radio_get_version(uint8_t *major, uint8_t *minor)
{
    return (int)lr2021_get_version(major, minor);
}

/** @brief Запуск передачи (не блокирует). Завершение: BSP_RADIO_EVT_TX_DONE из bsp_radio_poll(). */
int bsp_radio_tx(const uint8_t *data, uint16_t len)
{
    return (int)lr2021_tx(data, len);
}

/** @brief Запуск непрерывного приёма (не блокирует). */
int bsp_radio_rx_start(void)
{
    return (int)lr2021_rx_start();
}

/** @brief Остановить приём/передачу, перейти в режим ожидания. */
int bsp_radio_standby(void)
{
    return (int)lr2021_standby();
}

/**
 * @brief Проверить радио и получить накопившиеся события. Вызывать из основного цикла.
 *
 * SPI-обмен с чипом происходит только если на ноге IRQ был фронт или она сейчас в high.
 * Проверка уровня нужна на случай, когда новое событие пришло, пока линия ещё была
 * высокой: фронта нет, но событие не потеряется.
 *
 * @param events [out] маска BSP_RADIO_EVT_* (0, если событий нет)
 * @return 0 = успех (в том числе когда событий нет), иначе код ошибки
 */
int bsp_radio_poll(uint32_t *events)
{
    uint32_t chip_events = 0u;
    int      rc;

    if (events == NULL) {
        return (int)LR2021_ERR_PARAM;
    }
    *events = 0u;

    /* Порядок важен: сначала снять флаг, потом смотреть уровень */
    if ((bsp_radio_irq_take() == 0u) && (bsp_radio_irq_read() == 0u)) {
        return 0;
    }

    rc = (int)lr2021_irq_process(&chip_events);
    if (rc != 0) {
        return rc;
    }

    *events = prv_map_events(chip_events);
    return 0;
}

/**
 * @brief Прочитать принятый пакет (после BSP_RADIO_EVT_RX_DONE).
 * @param len [out] длина пакета; при коде -4 (буфер мал) тоже заполняется, пакет отбрасывается
 */
int bsp_radio_read_packet(uint8_t *buf, uint16_t buf_len, uint16_t *len)
{
    return (int)lr2021_read_packet(buf, buf_len, len);
}

/** @brief RSSI последнего принятого пакета, дБм. */
int bsp_radio_get_rssi(int16_t *rssi_dbm)
{
    return (int)lr2021_get_packet_rssi(rssi_dbm);
}

/**
 * @brief Счётчики приёма из чипа (16 бит, переполняются). Любой из указателей может быть NULL.
 */
int bsp_radio_get_rx_stats(uint16_t *received, uint16_t *crc_errors, uint16_t *len_errors)
{
    lr2021_rx_stats_t s;
    int               rc = (int)lr2021_get_rx_stats(&s);

    if (rc != 0) {
        return rc;
    }
    if (received != NULL)   { *received   = s.received; }
    if (crc_errors != NULL) { *crc_errors = s.crc_errors; }
    if (len_errors != NULL) { *len_errors = s.len_errors; }
    return 0;
}

/** @brief Обнулить счётчики приёма в чипе. */
int bsp_radio_reset_rx_stats(void)
{
    return (int)lr2021_reset_rx_stats();
}

/** @brief Прочитать и сбросить флаги ошибок чипа (при BSP_RADIO_EVT_CHIP_ERROR). */
int bsp_radio_get_errors(uint16_t *errors)
{
    return (int)lr2021_get_errors(errors);
}

/* ------------------------------------------------------------------------- */
/* UART2 (лог)                                                               */
/* ------------------------------------------------------------------------- */

int bsp_uart_write(const uint8_t *data, uint16_t len)
{
    if (len == 0u) {
        return 0;
    }
    if (data == NULL) {
        return -1;
    }
    return prv_hal_rc(HAL_UART_Transmit(&huart2, (uint8_t *)data, len, BSP_UART_TIMEOUT_MS));
}

/**
 * @brief Точка вывода для printf (newlib: syscalls.c -> _write -> __io_putchar).
 *        Имя фиксировано toolchain-ом, поэтому без префикса bsp_.
 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;

    (void)bsp_uart_write(&c, 1u);
    return ch;
}
