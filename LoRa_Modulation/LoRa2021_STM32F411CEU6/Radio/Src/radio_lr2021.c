#include "radio.h"

#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>

#include "board.h"
#include "lr20xx_hal.h"
#include "lr20xx_system.h"
#include "lr20xx_regmem.h"
#include "lr20xx_pram_load.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_fifo.h"

/* ---- Параметры радиоканала -------------------------------------------- */
/* У передатчика и приёмника должны совпадать: частота, SF, BW, CR, LDRO,
 * длина преамбулы, синхрослово. */

#define LORA_FREQ_HZ           2440000000u
#define LORA_SF                LR20XX_RADIO_LORA_SF7
#define LORA_BW                LR20XX_RADIO_LORA_BW_406
#define LORA_CR                LR20XX_RADIO_LORA_CR_4_5
#define LORA_LDRO              LR20XX_RADIO_LORA_PPM_1_4  /* даташит 9.9.1: для BW 203/406/812 включать */
#define LORA_PREAMBLE_SYMBOLS  12u
/* При приёме payload_len - максимум допустимой длины: длиннее пакеты чип отбрасывает как ошибку заголовка */
#define LORA_RX_MAX_PAYLOAD_BYTES      255u
#define LORA_PAYLOAD_BYTES     16u
#define LORA_SYNCWORD          0x12u                      /* 0x12 приватная сеть, 0x34 публичная */

/* ---- Мощность --------------------------------------------------------- */
/* TX_POWER_HALF_DBM: шаг 0,5 дБ на выходе чипа (даташит 7.4.3). Это то же число, что
 * "Register Value" в таблице NiceRF (стр. 7): -18 даёт около 10,8 дБм на антенне при 5 В.
 * Для 2,4 ГГц NiceRF не рекомендует значения больше 4.
 * PA_HF_DUTY_CYCLE: допустимо 16...31. Значение 27 для -18 взято из таблицы Semtech
 * (USP, lr20xx_pa_pwr_cfg.h: чип -9 дБм). Для другой мощности подбирать заново. */

#define LORA_TX_POWER_HALF_DBM (-18)
#define LORA_PA_HF_DUTY_CYCLE  27u
#define LORA_PA_RAMP           LR20XX_RADIO_COMMON_RAMP_48_US

/* ---- Передача --------------------------------------------------------- */
/* LORA_TX_TIMEOUT_MS: защита на стороне чипа. Если передача не закончилась, чип прервёт её и
 * выдаст Timeout. Пакет 16 байт при SF7/406 кГц идёт десятки миллисекунд.
 * IRQ_WATCHDOG_MS: если линия IRQ молчит, через это время читаем статус чипа сами. */

#define LORA_RX_BOOST                  LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_4   /* для HF рекомендовано 4 (даташит 7.3.2) */
#define LORA_RX_CONTINUOUS_RTC         0xFFFFFFu   /* SetRx: принимать, пока не остановят (даташит 6.2.x) */
/* Чип при перегрузке входа отдаёт RSSI около -254 дБм (на практике: платы в 10 см друг от друга).
 * Приёмник чувствительнее -140 дБм не бывает, всё что слабее считаем признаком перегрузки. */
#define LORA_RSSI_OVERLOAD_DBM         (-200)
#define LORA_TX_TIMEOUT_MS     1000u
#define LR2021_IRQ_WATCHDOG_MS (LORA_TX_TIMEOUT_MS + 100u)

/* ---- Линия IRQ модуля ------------------------------------------------- */
/* К какому DIO чипа подключён вывод IRQ, в документации NiceRF не сказано (DIO5...8 заняты
 * усилителем). Проверкой на плате найдено: DIO9. LR2021_IRQ_PROBE = 1 включает проверку при
 * старте: DIO9 и DIO10 по очереди ставятся в 0 и 1, а на PB4 смотрим, какой из них повторяется.
 * DIO11 не трогаем: возможно, на нём внешний тактовый генератор 32 кГц. */

#define LR2021_IRQ_DIO         LR20XX_SYSTEM_DIO_9
#define LR2021_IRQ_PROBE       0

/* ---- Параметры модуля LoRa2021F33-2G4 --------------------------------- */

#define LR2021_TCXO_VOLTAGE            LR20XX_SYSTEM_TCXO_CTRL_3_3V
#define LR2021_TCXO_START_32MHZ_TICKS  60000u   /* 60000 * 31,25 нс = 1,875 мс */
#define LR2021_BUSY_TIMEOUT_MS         1000u

#define LR2021_FW_MAJOR                0x01u    /* LR2021: 01.18 (даташит, GetVersion) */
#define LR2021_FW_MINOR                0x18u

#define LR2021_PRAM_CHECK_ADDR         0x800FF8u
#define LR2021_PRAM_CHECK_VALUE        0x600DB002u

#define LOG(...)  printf("[lr2021] " __VA_ARGS__)

#define CTX       ((const void *)0)   /* единственный чип, контекст не нужен */

/* ---- HAL для драйвера Semtech ----------------------------------------- */

static bool radio_asleep;

static bool wait_busy(void)
{
    uint32_t t0 = board_get_ms();

    while (board_radio_busy()) {
        if ((board_get_ms() - t0) > LR2021_BUSY_TIMEOUT_MS) {
            return false;
        }
    }
    return true;
}

/* Перед каждой транзакцией: разбудить, если спит, и дождаться BUSY = 0 */
static bool radio_ready(void)
{
    if (radio_asleep) {
        board_radio_nss(false);
        board_delay_us(100);
        board_radio_nss(true);
        radio_asleep = false;
    }
    return wait_busy();
}

lr20xx_hal_status_t lr20xx_hal_reset(const void *context)
{
    (void)context;
    board_radio_nrst(false);
    board_delay_ms(1);
    board_radio_nrst(true);
    radio_asleep = false;
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup(const void *context)
{
    (void)context;
    board_radio_nss(false);
    board_delay_us(100);
    board_radio_nss(true);
    radio_asleep = false;
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                     const uint8_t *data, const uint16_t data_length)
{
    bool ok;

    (void)context;
    if (!radio_ready()) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    board_radio_nss(false);
    ok = board_radio_spi_write(command, command_length);
    if (ok && (data_length > 0u)) {
        ok = board_radio_spi_write(data, data_length);
    }
    board_radio_nss(true);

    if (!ok) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* SetSleepMode (0x0127): дальше радио спит, перед следующей командой будим */
    if ((command_length >= 2u) && (command[0] == 0x01u) && (command[1] == 0x27u)) {
        radio_asleep = true;
        board_delay_us(500);
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length)
{
    bool ok;

    (void)context;
    if (!radio_ready()) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* Фаза 1: команда */
    board_radio_nss(false);
    ok = board_radio_spi_write(command, command_length);
    board_radio_nss(true);
    if (!ok) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* Фаза 2: два байта статуса и данные */
    if (data_length > 0u) {
        uint8_t status[2];

        if (!wait_busy()) {
            return LR20XX_HAL_STATUS_ERROR;
        }
        board_radio_nss(false);
        ok = board_radio_spi_read(status, sizeof(status)) && board_radio_spi_read(data, data_length);
        board_radio_nss(true);
        if (!ok) {
            return LR20XX_HAL_STATUS_ERROR;
        }
    }
    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read(const void *context, uint8_t *data, const uint16_t data_length)
{
    bool ok;

    (void)context;
    if (!radio_ready()) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    board_radio_nss(false);
    ok = board_radio_spi_read(data, data_length);
    board_radio_nss(true);

    return ok ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void *context, const uint8_t *command,
                                                const uint16_t command_length, uint8_t *data,
                                                const uint16_t data_length)
{
    bool ok;

    (void)context;
    if (!radio_ready()) {
        return LR20XX_HAL_STATUS_ERROR;
    }

    board_radio_nss(false);
    ok = board_radio_spi_write(command, command_length);
    if (ok && (data_length > 0u)) {
        ok = board_radio_spi_read(data, data_length);
    }
    board_radio_nss(true);

    return ok ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

/* ---- Запуск чипа ------------------------------------------------------ */
/* Сброс, версия, патч PRAM, TCXO, калибровка. После этого чип в Standby RC. */

static radio_status_t chip_start(void)
{
    lr20xx_system_version_t ver;
    lr20xx_system_errors_t  errors = 0;
    uint32_t                pram[2];

    /* Питание модуля и сброс */
    board_radio_ce(true);
    board_delay_ms(10);
    if (lr20xx_system_reset(CTX) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }

    /* Версия: заодно проверка, что чип отвечает по SPI */
    if (lr20xx_system_get_version(CTX, &ver) != LR20XX_STATUS_OK) {
        LOG("no answer from chip (BUSY stuck or SPI error)\r\n");
        return RADIO_ERR_COMM;
    }
    LOG("chip firmware version %02X.%02X\r\n", ver.major, ver.minor);
    if (ver.major != LR2021_FW_MAJOR) {
        LOG("not an LR2021 (expected major %02X)\r\n", LR2021_FW_MAJOR);
        return RADIO_ERR_CHIP;
    }
    if (ver.minor != LR2021_FW_MINOR) {
        LOG("note: datasheet lists minor %02X\r\n", LR2021_FW_MINOR);
    }

    /* Патч прошивки (PRAM): грузится после каждого сброса */
    if (lr20xx_pram_load_pram_lr2021(CTX) != LR20XX_STATUS_OK) {
        LOG("PRAM load failed\r\n");
        return RADIO_ERR_COMM;
    }
    if (lr20xx_regmem_read_regmem32(CTX, LR2021_PRAM_CHECK_ADDR, pram, 2) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    if (pram[0] != LR2021_PRAM_CHECK_VALUE) {
        LOG("PRAM check failed: 0x%08lX\r\n", (unsigned long)pram[0]);
        return RADIO_ERR_PRAM;
    }
    LOG("PRAM loaded, version %lu\r\n", (unsigned long)((pram[1] >> 8) & 0xFFFFu));

    /* TCXO и калибровка (калибровать нужно после запуска TCXO) */
    if (lr20xx_system_set_tcxo_mode(CTX, LR2021_TCXO_VOLTAGE, LR2021_TCXO_START_32MHZ_TICKS) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    if (lr20xx_system_clear_errors(CTX) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    if (lr20xx_system_calibrate(CTX, LR20XX_SYSTEM_CALIB_LF_RC_MASK | LR20XX_SYSTEM_CALIB_HF_RC_MASK |
                                         LR20XX_SYSTEM_CALIB_PLL_MASK | LR20XX_SYSTEM_CALIB_AAF_MASK |
                                         LR20XX_SYSTEM_CALIB_MU_MASK | LR20XX_SYSTEM_CALIB_PA_OFF_MASK) !=
        LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    if (lr20xx_system_get_errors(CTX, &errors) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    if (errors != 0u) {
        LOG("chip errors after calibration: 0x%04X\r\n", (unsigned)errors);
        if ((errors & LR20XX_SYSTEM_ERRORS_HF_XOSC_START_MASK) != 0u) {
            return RADIO_ERR_CLOCK;
        }
    }

    return RADIO_OK;
}

/* ---- Настройка передачи ----------------------------------------------- */

/* Вызов драйвера: при любой ошибке выходим из функции с RADIO_ERR_COMM */
#define TRY(call)                              \
    do {                                       \
        if ((call) != LR20XX_STATUS_OK) {      \
            return RADIO_ERR_COMM;             \
        }                                      \
    } while (0)

/* Ножки DIO чипа, которыми управляется усилитель модуля (блок-схема NiceRF, стр. 4).
 * high_in_modes: в каких режимах чипа на ножке единица. */
typedef struct {
    lr20xx_system_dio_t               dio;
    lr20xx_system_dio_drive_t         pull;
    lr20xx_system_dio_rf_switch_cfg_t high_in_modes;
} dio_switch_t;

#define MODE_STANDBY  LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_STANDBY
#define MODE_RX_HF    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF
#define MODE_TX_HF    LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF

static const dio_switch_t fem_pins[] = {
    /* DIO5, LNA_EN: 1 = малошумящий усилитель включён (NiceRF, стр. 5). Чип принимает для DIO5
     * только подтяжку вверх. */
    { LR20XX_SYSTEM_DIO_5, LR20XX_SYSTEM_DIO_DRIVE_PULL_UP, MODE_RX_HF },
    /* DIO6, TXEN суб-ГГц усилителя: не нужен, держим 0 (по умолчанию на DIO6 подтяжка к 1) */
    { LR20XX_SYSTEM_DIO_6, LR20XX_SYSTEM_DIO_DRIVE_NONE, 0 },
    /* DIO7, Power_EN: питание усилителя 2,4 ГГц. Включено и в Standby, чтобы питание успело
     * установиться до старта передачи (время включения в документации не указано). Активный
     * уровень в документации не указан, принят 1. Если окажется 0, маска:
     * LR20XX_SYSTEM_DIO_RF_SWITCH_ALL_MASK ^ (MODE_RX_HF | MODE_TX_HF) */
    { LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_DIO_DRIVE_NONE, MODE_STANDBY | MODE_RX_HF | MODE_TX_HF },
    /* DIO8, TXEN усилителя 2,4 ГГц */
    { LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_DRIVE_NONE, MODE_TX_HF },
};

/* ---- Поиск DIO, к которому подключён вывод IRQ модуля ---------------- */

static lr20xx_system_dio_t irq_dio = LR2021_IRQ_DIO;

#if LR2021_IRQ_PROBE

/* Ждём, пока чип выполнит команду (BUSY снимется) */
static void settle(void)
{
    board_delay_us(100);
    (void)wait_busy();
}

/* true, если вывод IRQ повторяет состояние DIO, когда оно работает как обычный выход */
static bool dio_drives_irq_pin(lr20xx_system_dio_t dio)
{
    bool high;
    bool low;

    (void)lr20xx_system_set_dio_function(CTX, dio, LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH, LR20XX_SYSTEM_DIO_DRIVE_NONE);
    settle();
    high = board_radio_irq();

    (void)lr20xx_system_set_dio_function(CTX, dio, LR20XX_SYSTEM_DIO_FUNC_GPIO_LOW, LR20XX_SYSTEM_DIO_DRIVE_NONE);
    settle();
    low = board_radio_irq();

    (void)lr20xx_system_set_dio_function(CTX, dio, LR20XX_SYSTEM_DIO_FUNC_NONE, LR20XX_SYSTEM_DIO_DRIVE_NONE);
    settle();

    return high && !low;
}

static void irq_probe(void)
{
    static const lr20xx_system_dio_t candidates[] = { LR20XX_SYSTEM_DIO_9, LR20XX_SYSTEM_DIO_10 };
    uint8_t                          i;

    if (dio_drives_irq_pin(irq_dio)) {
        LOG("IRQ pin is connected to DIO%u\r\n", (unsigned)irq_dio);
        return;
    }
    for (i = 0; i < (sizeof(candidates) / sizeof(candidates[0])); i++) {
        if ((candidates[i] != irq_dio) && dio_drives_irq_pin(candidates[i])) {
            irq_dio = candidates[i];
            LOG("IRQ pin is connected to DIO%u (set LR2021_IRQ_DIO)\r\n", (unsigned)irq_dio);
            return;
        }
    }
    LOG("IRQ pin does not follow DIO9/DIO10 (DIO11?), using DIO%u\r\n", (unsigned)irq_dio);
}

#endif /* LR2021_IRQ_PROBE */

/* ---- Конфигурация LoRa 2,4 ГГц для передачи --------------------------- */

/* Длина, с которой сейчас настроены параметры пакета. Для передачи она должна совпадать
 * с числом байт в FIFO, поэтому при другой длине параметры перенастраиваются. */
static uint8_t pkt_len_cfg;

static bool     tx_active;     /* передача запущена и ещё не завершена */
static bool     rx_active;     /* запущен непрерывный приём */
static uint32_t tx_start_ms;

static lr20xx_status_t set_packet_params(uint8_t len)
{
    const lr20xx_radio_lora_pkt_params_t pkt = {
        .preamble_len_in_symb = LORA_PREAMBLE_SYMBOLS,
        .pld_len_in_bytes     = len,
        .pkt_mode             = LR20XX_RADIO_LORA_PKT_EXPLICIT,
        .crc                  = LR20XX_RADIO_LORA_CRC_ENABLED,
        .iq                   = LR20XX_RADIO_LORA_IQ_STANDARD,
    };
    lr20xx_status_t st = lr20xx_radio_lora_set_packet_params(CTX, &pkt);

    if (st == LR20XX_STATUS_OK) {
        pkt_len_cfg = len;
    }
    return st;
}

static radio_status_t tx_config(void)
{
    lr20xx_system_irq_mask_t irq    = 0;
    lr20xx_system_errors_t   errors = 0;
    uint8_t                  i;

    const lr20xx_radio_common_pa_cfg_t pa = {
        .pa_sel           = LR20XX_RADIO_COMMON_PA_SEL_HF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 6,   /* LF-усилитель не используется: значения из даташита 7.4.1 */
        .pa_lf_slices     = 7,
        .pa_hf_duty_cycle = LORA_PA_HF_DUTY_CYCLE,
    };
    const lr20xx_radio_lora_mod_params_t mod = {
        .sf  = LORA_SF,
        .bw  = LORA_BW,
        .cr  = LORA_CR,
        .ppm = LORA_LDRO,
    };

    irq_dio = LR2021_IRQ_DIO;   /* при повторной инициализации начинаем с заданного номера */

    /* Чистый лист: после настройки по флагу CMD_ERROR узнаем, отклонил ли чип команду */
    TRY(lr20xx_system_clear_irq_status(CTX, LR20XX_SYSTEM_IRQ_ALL_MASK));

#if LR2021_IRQ_PROBE
    irq_probe();
#endif

    /* Усилитель модуля */
    for (i = 0; i < (sizeof(fem_pins) / sizeof(fem_pins[0])); i++) {
        TRY(lr20xx_system_set_dio_function(CTX, fem_pins[i].dio, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                           fem_pins[i].pull));
        TRY(lr20xx_system_set_dio_rf_switch_cfg(CTX, fem_pins[i].dio, fem_pins[i].high_in_modes));
    }

    /* Линия IRQ: сигнал по окончании передачи и по таймауту */
    TRY(lr20xx_system_set_dio_function(CTX, irq_dio, LR20XX_SYSTEM_DIO_FUNC_IRQ, LR20XX_SYSTEM_DIO_DRIVE_NONE));
    TRY(lr20xx_system_set_dio_irq_cfg(CTX, irq_dio, LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT));

    /* Радио. SetPacketType всегда первым, SetPaConfig раньше SetTxParams */
    TRY(lr20xx_radio_common_set_pkt_type(CTX, LR20XX_RADIO_COMMON_PKT_TYPE_LORA));
    TRY(lr20xx_radio_common_set_rf_freq(CTX, LORA_FREQ_HZ));
    TRY(lr20xx_radio_common_set_pa_cfg(CTX, &pa));
    TRY(lr20xx_radio_common_set_tx_params(CTX, LORA_TX_POWER_HALF_DBM, LORA_PA_RAMP));
    TRY(lr20xx_radio_lora_set_modulation_params(CTX, &mod));
    TRY(set_packet_params(LORA_PAYLOAD_BYTES));
    TRY(lr20xx_radio_lora_set_syncword(CTX, LORA_SYNCWORD));

    /* Проверка: чип не отклонил ни одной команды настройки */
    TRY(lr20xx_system_get_and_clear_irq_status(CTX, &irq));
    if ((irq & LR20XX_SYSTEM_IRQ_CMD_ERROR) != 0u) {
        LOG("chip rejected a TX setup command\r\n");
        return RADIO_ERR_CONFIG;
    }
    TRY(lr20xx_system_get_errors(CTX, &errors));
    if (errors != 0u) {
        LOG("chip errors after TX setup: 0x%04X\r\n", (unsigned)errors);
    }

    LOG("TX ready: %lu Hz, SF%u, power %d (x0.5 dB), IRQ on DIO%u\r\n", (unsigned long)LORA_FREQ_HZ,
        (unsigned)LORA_SF, (int)LORA_TX_POWER_HALF_DBM, (unsigned)irq_dio);
    return RADIO_OK;
}

/* ---- Конфигурация приёма ---------------------------------------------- */

/* Чип в Standby, команды общей части (пин FEM, тип пакета, частота, модуляция, синхрослово)
 * уже выданы в tx_config(). Здесь только то, что нужно приёму: тракт HF, калибровка
 * входного тракта на рабочей частоте и маска прерываний. SetRx здесь не вызывается. */
static radio_status_t rx_config(void)
{
    lr20xx_system_irq_mask_t irq    = 0;
    lr20xx_system_errors_t   errors = 0;

    const lr20xx_radio_common_front_end_calibration_value_t calib = {
        .rx_path            = LR20XX_RADIO_COMMON_RX_PATH_HF,
        .frequency_in_hertz = LORA_FREQ_HZ,
    };

    TRY(lr20xx_system_clear_irq_status(CTX, LR20XX_SYSTEM_IRQ_ALL_MASK));

    /* Линия IRQ: пакет принят, таймаут, ошибка CRC, ошибка заголовка */
    TRY(lr20xx_system_set_dio_irq_cfg(CTX, irq_dio, LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT |
                                                        LR20XX_SYSTEM_IRQ_CRC_ERROR |
                                                        LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR));

    /* Тракт HF (2,4 ГГц) и усиление; LNA модуля включается выводом DIO5 в режиме RX_HF */
    TRY(lr20xx_radio_common_set_rx_path(CTX, LR20XX_RADIO_COMMON_RX_PATH_HF, LORA_RX_BOOST));
    TRY(set_packet_params(LORA_RX_MAX_PAYLOAD_BYTES));

    /* После сброса калибровки входного тракта нет: без неё SetRx даёт ошибку RxFreqNoCalErr.
     * CalibFE не работает в режимах RX/TX, поэтому только из Standby. */
    TRY(lr20xx_radio_common_calibrate_front_end_helper(CTX, &calib, 1));

    TRY(lr20xx_system_get_and_clear_irq_status(CTX, &irq));
    if ((irq & LR20XX_SYSTEM_IRQ_CMD_ERROR) != 0u) {
        LOG("chip rejected an RX setup command\r\n");
        return RADIO_ERR_CONFIG;
    }
    TRY(lr20xx_system_get_errors(CTX, &errors));
    if (errors != 0u) {
        LOG("chip errors after RX setup: 0x%04X\r\n", (unsigned)errors);
    }
    LOG("RX ready: %lu Hz, SF%u, HF path, boost %u\r\n", (unsigned long)LORA_FREQ_HZ, (unsigned)LORA_SF,
        (unsigned)LORA_RX_BOOST);
    return RADIO_OK;
}

/* ---- Интерфейс radio.h ------------------------------------------------ */

radio_status_t radio_lora_init(void)
{
    radio_status_t st = chip_start();

    if (st != RADIO_OK) {
        return st;
    }
    st = tx_config();
    if (st != RADIO_OK) {
        return st;
    }
    tx_active = false;
    rx_active = false;
    LOG("init done\r\n");
    return RADIO_OK;
}

radio_status_t radio_lora_rx_config(void)
{
    if (tx_active || rx_active) {
        return RADIO_ERR_BUSY;   /* CalibFE не работает в режимах RX и TX */
    }
    return rx_config();
}

/* ---- Передача пакета -------------------------------------------------- */

radio_status_t radio_lora_send(const uint8_t *data, uint8_t len)
{
    if (tx_active || rx_active) {
        return RADIO_ERR_BUSY;
    }
    if ((data == NULL) || (len == 0u)) {
        return RADIO_ERR_CONFIG;
    }

    if (len != pkt_len_cfg) {
        TRY(set_packet_params(len));
    }
    TRY(lr20xx_radio_fifo_clear_tx(CTX));                                   /* остатки прерванной передачи */
    TRY(lr20xx_radio_fifo_write_tx(CTX, data, len));
    TRY(lr20xx_system_clear_irq_status(CTX, LR20XX_SYSTEM_IRQ_ALL_MASK));   /* сбросить старые флаги и линию IRQ */
    (void)board_radio_irq_take();                                           /* и устаревший флаг прерывания */
    TRY(lr20xx_radio_common_set_tx(CTX, LORA_TX_TIMEOUT_MS));

    tx_start_ms = board_get_ms();
    tx_active   = true;
    return RADIO_OK;
}

radio_tx_result_t radio_lora_tx_poll(void)
{
    lr20xx_system_irq_mask_t irq = 0;
    bool                     watchdog;

    if (!tx_active) {
        return RADIO_TX_ERROR;
    }

    /* Статус читаем по прерыванию с вывода IRQ. Если оно не пришло, ждём сторожевой интервал
     * и читаем сами: линия IRQ могла быть подключена не к тому DIO. */
    watchdog = (board_get_ms() - tx_start_ms) >= LR2021_IRQ_WATCHDOG_MS;
    if (!board_radio_irq_take() && !watchdog) {
        return RADIO_TX_BUSY;
    }

    if (lr20xx_system_get_and_clear_irq_status(CTX, &irq) != LR20XX_STATUS_OK) {
        tx_active = false;
        return RADIO_TX_ERROR;
    }

    if ((irq & LR20XX_SYSTEM_IRQ_CMD_ERROR) != 0u) {
        LOG("chip rejected SetTx\r\n");
        tx_active = false;
        return RADIO_TX_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_PA_OVP_OCP) != 0u) {
        LOG("PA over-current / over-voltage protection triggered\r\n");
        tx_active = false;
        return RADIO_TX_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_TX_DONE) != 0u) {
        tx_active = false;
        return RADIO_TX_DONE;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_TIMEOUT) != 0u) {
        tx_active = false;
        return RADIO_TX_TIMEOUT;
    }
    if (watchdog) {
        LOG("no TX_DONE and no IRQ pulse within %u ms\r\n", (unsigned)LR2021_IRQ_WATCHDOG_MS);
        tx_active = false;
        return RADIO_TX_TIMEOUT;
    }
    return RADIO_TX_BUSY;   /* прерывание было по другой причине, передача продолжается */
}

/* ---- Приём пакетов ---------------------------------------------------- */

radio_status_t radio_lora_rx_start(void)
{
    if (tx_active || rx_active) {
        return RADIO_ERR_BUSY;
    }

    TRY(lr20xx_radio_fifo_clear_rx(CTX));                                   /* остатки прошлого приёма */
    TRY(lr20xx_system_clear_irq_status(CTX, LR20XX_SYSTEM_IRQ_ALL_MASK));
    (void)board_radio_irq_take();
    TRY(lr20xx_radio_common_set_rx_with_timeout_in_rtc_step(CTX, LORA_RX_CONTINUOUS_RTC));

    rx_active = true;
    return RADIO_OK;
}

/* Читает принятый пакет и его параметры. Вызывается после RX_DONE. */
static radio_rx_result_t rx_read_packet(uint8_t *buf, uint8_t buf_size, radio_rx_info_t *info)
{
    lr20xx_radio_lora_packet_status_t st;
    uint16_t                          len = 0;
    uint16_t                          n;

    if (lr20xx_radio_common_get_rx_packet_length(CTX, &len) != LR20XX_STATUS_OK) {
        return RADIO_RX_ERROR;
    }
    n = (len > buf_size) ? buf_size : len;
    if ((n > 0u) && (lr20xx_radio_fifo_read_rx(CTX, buf, n) != LR20XX_STATUS_OK)) {
        return RADIO_RX_ERROR;
    }
    if (len > n) {
        LOG("packet %u bytes does not fit in %u, tail dropped\r\n", (unsigned)len, (unsigned)buf_size);
        if (lr20xx_radio_fifo_clear_rx(CTX) != LR20XX_STATUS_OK) {
            return RADIO_RX_ERROR;
        }
    }
    if (lr20xx_radio_lora_get_packet_status(CTX, &st) != LR20XX_STATUS_OK) {
        return RADIO_RX_ERROR;
    }

    info->len            = (uint8_t)n;
    info->rssi_half_dbm  = (int16_t)(2 * st.rssi_pkt_in_dbm - (int16_t)st.rssi_pkt_half_dbm_count);
    info->snr_quarter_db = st.snr_pkt_raw;
    info->rssi_overload  = (st.rssi_pkt_in_dbm < LORA_RSSI_OVERLOAD_DBM);
    return RADIO_RX_DONE;
}

radio_rx_result_t radio_lora_rx_poll(uint8_t *buf, uint8_t buf_size, radio_rx_info_t *info)
{
    lr20xx_system_irq_mask_t irq = 0;
    radio_rx_result_t        res;

    if (!rx_active || (buf == NULL) || (info == NULL)) {
        return RADIO_RX_ERROR;
    }
    if (!board_radio_irq_take()) {
        return RADIO_RX_NONE;
    }

    if (lr20xx_system_get_and_clear_irq_status(CTX, &irq) != LR20XX_STATUS_OK) {
        rx_active = false;
        return RADIO_RX_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_CMD_ERROR) != 0u) {
        LOG("chip rejected SetRx\r\n");
        rx_active = false;
        return RADIO_RX_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_TIMEOUT) != 0u) {
        LOG("unexpected RX timeout, receiver stopped\r\n");
        rx_active = false;
        return RADIO_RX_ERROR;
    }
    /* Пакет с ошибкой CRC тоже может прийти с RX_DONE: данные всё равно не отдаём */
    if ((irq & LR20XX_SYSTEM_IRQ_CRC_ERROR) != 0u) {
        if (lr20xx_radio_fifo_clear_rx(CTX) != LR20XX_STATUS_OK) {
            rx_active = false;
            return RADIO_RX_ERROR;
        }
        return RADIO_RX_CRC_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR) != 0u) {
        return RADIO_RX_HEADER_ERROR;
    }
    if ((irq & LR20XX_SYSTEM_IRQ_RX_DONE) != 0u) {
        res = rx_read_packet(buf, buf_size, info);
        if (res == RADIO_RX_ERROR) {
            rx_active = false;
        }
        return res;
    }
    return RADIO_RX_NONE;
}

radio_status_t radio_lora_get_version(uint8_t *major, uint8_t *minor)
{
    lr20xx_system_version_t ver;

    if (lr20xx_system_get_version(CTX, &ver) != LR20XX_STATUS_OK) {
        return RADIO_ERR_COMM;
    }
    *major = ver.major;
    *minor = ver.minor;
    return RADIO_OK;
}
