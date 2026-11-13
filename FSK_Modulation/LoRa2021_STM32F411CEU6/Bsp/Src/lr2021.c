/**
 * @file    lr2021.c
 * @brief   Слой радиочипа LR2021: инициализация, FSK-конфигурация, передача, приём.
 *
 * Использует драйвер Semtech (Lib/lr20xx_driver) и bsp_* из board.h.
 * Ничего не печатает, все результаты возвращаются вызывающему слою.
 * Параметры радио задаются в apps_configuration.h.
 */

#include <stdint.h>
#include <stddef.h>

#include "lr2021.h"
#include "board.h"
#include "apps_configuration.h"

#include "lr20xx_hal.h"
#include "lr20xx_system.h"
#include "lr20xx_pram_load.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_fsk.h"

/* ------------------------------------------------------------------------- */
/* Проверка параметров FSK на этапе компиляции (даташит LR20xx, 11.1.3)      */
/* ------------------------------------------------------------------------- */

/* Индекс модуляции h = 2*fdev/bitrate, умноженный на 100 (целая арифметика, 32 бита хватает) */
#define LR2021_H_X100       ( ( 2u * ( FSK_FDEV_HZ ) * 100u ) / ( FSK_BITRATE_BPS ) )

/* Коэффициент передискретизации демодулятора: h < 0.85 -> 4, иначе 8 */
#define LR2021_OSR          ( ( LR2021_H_X100 < 85u ) ? 4u : 8u )

/* Максимальный индекс модуляции: GFSK 6.2, обычный FSK 8.33 */
#define LR2021_H_X100_MAX   ( ( FSK_PULSE_SHAPE == LR20XX_RADIO_FSK_PULSE_SHAPE_DISABLED ) ? 833u : 620u )

_Static_assert( ( FSK_BITRATE_BPS ) >= 500u && ( FSK_BITRATE_BPS ) <= 2000000u,
                "FSK_BITRATE_BPS must be in range 500..2000000 bps" );
_Static_assert( ( FSK_FDEV_HZ ) >= 600u && ( FSK_FDEV_HZ ) <= 500000u,
                "FSK_FDEV_HZ must be in range 600..500000 Hz" );
_Static_assert( LR2021_H_X100 >= 30u && LR2021_H_X100 <= LR2021_H_X100_MAX,
                "Modulation index h = 2*fdev/bitrate must be in 0.3..6.2 (GFSK) or 0.3..8.33 (FSK)" );
_Static_assert( ( FSK_BITRATE_BPS ) * LR2021_OSR <= 8000000u,
                "bitrate * osr must not exceed 8 MHz: reduce bitrate or modulation index h" );
_Static_assert( ( FSK_RX_BW_MIN_HZ ) <= 3076900u,
                "Required RX bandwidth exceeds the 3076 kHz maximum: reduce bitrate or fdev" );
_Static_assert( ( FSK_RX_BW_MIN_HZ ) <= ( FSK_BITRATE_BPS ) * LR2021_OSR,
                "RX bandwidth must not exceed bitrate * osr" );
_Static_assert( ( FSK_MAX_PAYLOAD_BYTES ) >= 1u && ( FSK_MAX_PAYLOAD_BYTES ) <= 255u,
                "FSK_MAX_PAYLOAD_BYTES must be 1..255 (8-bit length header, 256-byte FIFO)" );
_Static_assert( ( FSK_SYNCWORD_BITS ) >= 1u && ( FSK_SYNCWORD_BITS ) <= 32u,
                "FSK_SYNCWORD_BITS must be 1..32 (FSK_SYNCWORD is a 32-bit value)" );

/* ------------------------------------------------------------------------- */
/* Приватные определения                                                     */
/* ------------------------------------------------------------------------- */

#define LR2021_CHECK( call )                      \
    do                                            \
    {                                             \
        if( ( call ) != LR20XX_STATUS_OK )        \
        {                                         \
            return LR2021_ERR_IO;                 \
        }                                         \
    } while( 0 )

/* Нога IRQ чипа, подключённая к МК (DIO9) */
#define LR2021_IRQ_DIO      LR20XX_SYSTEM_DIO_9

/* Прерывания, выведенные на ногу IRQ */
#define LR2021_IRQ_MASK                                                                         \
    ( LR20XX_SYSTEM_IRQ_TX_DONE | LR20XX_SYSTEM_IRQ_RX_DONE | LR20XX_SYSTEM_IRQ_TIMEOUT |        \
      LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR | LR20XX_SYSTEM_IRQ_ERROR |      \
      LR20XX_SYSTEM_IRQ_CMD_ERROR )

/* Флаги, которые считаются ошибкой чипа */
#define LR2021_IRQ_CHIP_ERR_MASK \
    ( LR20XX_SYSTEM_IRQ_ERROR | LR20XX_SYSTEM_IRQ_CMD_ERROR | LR20XX_SYSTEM_IRQ_PA_OVP_OCP )

/* Специальное значение таймаута RX: непрерывный приём */
#define LR2021_RX_CONTINUOUS    0xFFFFFFu

/* Старт чипа после включения TCXO, в шагах 32 МГц (64000 шагов = 2 мс) */
#define LR2021_TCXO_START_DELAY 64000u

/* Текущие параметры пакета; payload_length меняется при передаче */
static lr20xx_radio_fsk_pkt_params_t s_pkt_params;
static uint8_t                       s_pkt_params_valid = 0u;

/* ------------------------------------------------------------------------- */
/* Приватные функции                                                         */
/* ------------------------------------------------------------------------- */

/* Путь приёма зависит от диапазона: ниже 1.6 ГГц - LF, выше - HF */
static lr20xx_radio_common_rx_path_t prv_rx_path( uint32_t freq_in_hz )
{
    return ( freq_in_hz >= 1600000000u ) ? LR20XX_RADIO_COMMON_RX_PATH_HF : LR20XX_RADIO_COMMON_RX_PATH_LF;
}

/* Калибровка front-end на трёх частотах (поддиапазоны 430-510 МГц, 867-928 МГц, 2.4 ГГц) */
static lr2021_status_t prv_calibrate_front_end( void )
{
    static const uint32_t freq_in_hz[3] = { 470000000u, 897500000u, 2441000000u };

    lr20xx_radio_common_front_end_calibration_value_t cal[3] = { 0 };

    for( uint8_t i = 0; i < 3u; i++ )
    {
        cal[i].rx_path            = prv_rx_path( freq_in_hz[i] );
        cal[i].frequency_in_hertz = freq_in_hz[i];
    }

    LR2021_CHECK( lr20xx_radio_common_calibrate_front_end_helper( NULL, cal, 3 ) );
    return LR2021_OK;
}

/* Задать длину нагрузки в параметрах пакета, если она изменилась.
 * В variable-length режиме значение = максимальная длина для RX; для TX мы задаём
 * фактическую длину (так же делает пример для LoRa), чтобы не зависеть от того,
 * берёт ли чип длину передачи из параметров или из уровня TX FIFO. */
static lr2021_status_t prv_set_payload_length( uint16_t len )
{
    if( s_pkt_params_valid == 0u )
    {
        return LR2021_ERR_CONFIG; /* fsk_config() ещё не вызывали */
    }
    if( s_pkt_params.payload_length != len )
    {
        lr20xx_radio_fsk_pkt_params_t p = s_pkt_params;
        p.payload_length                = len;
        LR2021_CHECK( lr20xx_radio_fsk_set_packet_params( NULL, &p ) );
        s_pkt_params = p;
    }
    return LR2021_OK;
}

/* ------------------------------------------------------------------------- */
/* Публичные функции                                                         */
/* ------------------------------------------------------------------------- */

lr2021_status_t lr2021_init( void )
{
    s_pkt_params_valid = 0u;

    if( lr20xx_hal_reset( NULL ) != LR20XX_HAL_STATUS_OK )
    {
        return LR2021_ERR_IO;
    }
    bsp_radio_ce_set( 1u );

    LR2021_CHECK( lr20xx_pram_load_pram_lr2021( NULL ) );

    /* Питание: DC-DC; тактирование: внутренний RC 32 кГц для LF-часов */
    LR2021_CHECK( lr20xx_system_set_reg_mode( NULL, LR20XX_SYSTEM_REG_MODE_DCDC ) );
    LR2021_CHECK( lr20xx_system_cfg_lfclk( NULL, LR20XX_SYSTEM_LFCLK_RC ) );

    bsp_delay_ms( 10u );

    /* TCXO модуля питается от чипа, 3.3 В */
    LR2021_CHECK( lr20xx_system_set_standby_mode( NULL, LR20XX_SYSTEM_STANDBY_MODE_RC ) );
    LR2021_CHECK( lr20xx_system_set_tcxo_mode( NULL, LR20XX_SYSTEM_TCXO_CTRL_3_3V, LR2021_TCXO_START_DELAY ) );

    uint16_t errors = 0u;
    LR2021_CHECK( lr20xx_system_get_errors( NULL, &errors ) );
    if( errors != 0u )
    {
        LR2021_CHECK( lr20xx_system_clear_errors( NULL ) );
    }
    LR2021_CHECK( lr20xx_system_clear_irq_status( NULL, LR20XX_SYSTEM_IRQ_ALL_MASK ) );

    /* IRQ -> DIO9 */
    LR2021_CHECK( lr20xx_system_set_dio_function( NULL, LR2021_IRQ_DIO, LR20XX_SYSTEM_DIO_FUNC_IRQ,
                                                  LR20XX_SYSTEM_DIO_DRIVE_PULL_UP ) );
    LR2021_CHECK( lr20xx_system_set_dio_irq_cfg( NULL, LR2021_IRQ_DIO, LR2021_IRQ_MASK ) );

    /* RF-ключи: DIO6 - TX sub-GHz, DIO8 - TX 2.4 ГГц, DIO5 - LNA 2.4 ГГц (RX), DIO7 - питание PA 2.4 ГГц */
    LR2021_CHECK( lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_6, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE ) );
    LR2021_CHECK( lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_8, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                                  LR20XX_SYSTEM_DIO_DRIVE_NONE ) );
    LR2021_CHECK( lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_7, LR20XX_SYSTEM_DIO_FUNC_GPIO_HIGH,
                                                  LR20XX_SYSTEM_DIO_DRIVE_PULL_UP ) );
    LR2021_CHECK( lr20xx_system_set_dio_rf_switch_cfg( NULL, LR20XX_SYSTEM_DIO_6,
                                                       LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_LF ) );
    LR2021_CHECK( lr20xx_system_set_dio_rf_switch_cfg( NULL, LR20XX_SYSTEM_DIO_8,
                                                       LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_TX_HF ) );
    LR2021_CHECK( lr20xx_system_set_dio_function( NULL, LR20XX_SYSTEM_DIO_5, LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH,
                                                  LR20XX_SYSTEM_DIO_DRIVE_PULL_UP ) );
    LR2021_CHECK( lr20xx_system_set_dio_rf_switch_cfg( NULL, LR20XX_SYSTEM_DIO_5,
                                                       LR20XX_SYSTEM_DIO_RF_SWITCH_WHEN_RX_HF ) );

    LR2021_CHECK( lr20xx_system_cfg_clk_output( NULL, LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ ) );

    return prv_calibrate_front_end( );
}

lr2021_status_t lr2021_get_version( uint8_t *major, uint8_t *minor )
{
    lr20xx_system_version_t version;

    if( ( major == NULL ) || ( minor == NULL ) )
    {
        return LR2021_ERR_PARAM;
    }
    LR2021_CHECK( lr20xx_system_get_version( NULL, &version ) );
    *major = version.major;
    *minor = version.minor;
    return LR2021_OK;
}

lr2021_status_t lr2021_fsk_config( void )
{
    const uint32_t freq = RF_FREQ_IN_HZ;

    s_pkt_params_valid = 0u;

    LR2021_CHECK( lr20xx_radio_common_set_pkt_type( NULL, LR20XX_RADIO_COMMON_PKT_TYPE_FSK ) );
    LR2021_CHECK( lr20xx_radio_common_set_rf_freq( NULL, freq ) );

    /* Усилитель мощности: sub-GHz - LF PA, 1.9 ГГц и выше - HF PA */
    lr20xx_radio_common_pa_cfg_t pa_cfg = {
        .pa_sel           = ( freq < 1900000000u ) ? LR20XX_RADIO_COMMON_PA_SEL_LF : LR20XX_RADIO_COMMON_PA_SEL_HF,
        .pa_lf_mode       = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM,
        .pa_lf_duty_cycle = 7,
        .pa_lf_slices     = 6,
        .pa_hf_duty_cycle = 16
    };
    LR2021_CHECK( lr20xx_radio_common_set_pa_cfg( NULL, &pa_cfg ) );
    LR2021_CHECK( lr20xx_radio_common_set_tx_params( NULL, TX_OUTPUT_POWER_HALF_DBM_STEPS, PA_RAMP_TIME ) );
    LR2021_CHECK( lr20xx_radio_common_set_rx_path( NULL, prv_rx_path( freq ),
                                                   LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE ) );
    LR2021_CHECK( lr20xx_radio_common_set_rx_tx_fallback_mode( NULL, FALLBACK_MODE ) );

    /* Модуляция: полоса приёма - ближайшая табличная не меньше FSK_RX_BW_MIN_HZ */
    lr20xx_radio_fsk_mod_params_t mod = {
        .bitrate_unit = LR20XX_RADIO_FSK_MOD_PARAMS_BR_IN_BPS,
        .bitrate      = FSK_BITRATE_BPS,
        .pulse_shape  = FSK_PULSE_SHAPE,
        .fdev_in_hz   = FSK_FDEV_HZ,
    };
    if( lr20xx_radio_fsk_get_rx_bandwidth( FSK_RX_BW_MIN_HZ, &mod.bw ) != LR20XX_STATUS_OK )
    {
        return LR2021_ERR_CONFIG;
    }
    LR2021_CHECK( lr20xx_radio_fsk_set_modulation_params( NULL, &mod ) );

    /* Whitening и CRC должны быть заданы до пакетных параметров, которые их включают */
    LR2021_CHECK( lr20xx_radio_fsk_set_whitening_params(
        NULL, LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX126X_LR11XX, FSK_WHITENING_SEED ) );
    LR2021_CHECK( lr20xx_radio_fsk_set_crc_params( NULL, FSK_CRC_POLYNOMIAL, FSK_CRC_SEED ) );

    /* Синхрослово: 64-битная переменная, значимые биты - младшие */
    const uint32_t sw = FSK_SYNCWORD;
    const uint8_t  syncword[LR20XX_RADIO_FSK_SYNCWORD_LENGTH] = {
        0u, 0u, 0u, 0u, ( uint8_t )( sw >> 24 ), ( uint8_t )( sw >> 16 ), ( uint8_t )( sw >> 8 ), ( uint8_t ) sw
    };
    LR2021_CHECK( lr20xx_radio_fsk_set_syncword( NULL, syncword, FSK_SYNCWORD_BITS, LR20XX_RADIO_FSK_SYNCWORD_MSBF ) );

    /* Формат пакета: преамбула | синхрослово | заголовок длины 8 бит | нагрузка | CRC */
    lr20xx_radio_fsk_pkt_params_t pkt = {
        .pbl_length_in_bit     = FSK_PREAMBLE_TX_BITS,
        .preamble_detector     = FSK_PREAMBLE_DETECTOR,
        .long_preamble_enabled = false,
        .address_filtering     = LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED,
        .header_mode           = LR20XX_RADIO_FSK_HEADER_8BITS,
        .payload_length_unit   = LR20XX_RADIO_FSK_PAYLOAD_LENGTH_IN_BYTE,
        .payload_length        = FSK_MAX_PAYLOAD_BYTES,
        .crc                   = FSK_CRC_TYPE,
        .whitening             = FSK_WHITENING,
    };
    LR2021_CHECK( lr20xx_radio_fsk_set_packet_params( NULL, &pkt ) );

    s_pkt_params       = pkt;
    s_pkt_params_valid = 1u;
    return LR2021_OK;
}

lr2021_status_t lr2021_standby( void )
{
    LR2021_CHECK( lr20xx_system_set_standby_mode( NULL, LR20XX_SYSTEM_STANDBY_MODE_RC ) );
    return LR2021_OK;
}

lr2021_status_t lr2021_tx( const uint8_t *data, uint16_t len )
{
    if( ( data == NULL ) || ( len == 0u ) || ( len > FSK_MAX_PAYLOAD_BYTES ) )
    {
        return LR2021_ERR_PARAM;
    }

    /* Выход из RX (если был), чистые флаги и FIFO */
    LR2021_CHECK( lr20xx_system_set_standby_mode( NULL, LR20XX_SYSTEM_STANDBY_MODE_RC ) );
    LR2021_CHECK( lr20xx_system_clear_irq_status( NULL, LR20XX_SYSTEM_IRQ_ALL_MASK ) );
    LR2021_CHECK( lr20xx_radio_fifo_clear_tx( NULL ) );

    const lr2021_status_t st = prv_set_payload_length( len );
    if( st != LR2021_OK )
    {
        return st;
    }

    LR2021_CHECK( lr20xx_radio_fifo_write_tx( NULL, data, len ) );
    LR2021_CHECK( lr20xx_radio_common_set_tx( NULL, FSK_TX_TIMEOUT_MS ) );
    return LR2021_OK;
}

lr2021_status_t lr2021_rx_start( void )
{
    /* Для RX payload_length = максимально допустимая длина пакета */
    const lr2021_status_t st = prv_set_payload_length( FSK_MAX_PAYLOAD_BYTES );
    if( st != LR2021_OK )
    {
        return st;
    }

    /* FIFO чистим до запуска приёма, иначе можно стереть уже приходящие данные */
    LR2021_CHECK( lr20xx_system_clear_irq_status( NULL, LR20XX_SYSTEM_IRQ_ALL_MASK ) );
    LR2021_CHECK( lr20xx_radio_fifo_clear_rx( NULL ) );
    LR2021_CHECK( lr20xx_radio_common_set_rx_with_timeout_in_rtc_step( NULL, LR2021_RX_CONTINUOUS ) );
    return LR2021_OK;
}

lr2021_status_t lr2021_irq_process( uint32_t *events )
{
    lr20xx_system_irq_mask_t irq = 0u;
    uint32_t                 ev  = 0u;

    if( events == NULL )
    {
        return LR2021_ERR_PARAM;
    }
    *events = 0u;

    LR2021_CHECK( lr20xx_system_get_and_clear_irq_status( NULL, &irq ) );

    if( ( irq & LR20XX_SYSTEM_IRQ_TX_DONE ) != 0u )
    {
        ev |= LR2021_EVT_TX_DONE;
    }
    if( ( irq & LR20XX_SYSTEM_IRQ_TIMEOUT ) != 0u )
    {
        ev |= LR2021_EVT_TIMEOUT;
    }
    if( ( irq & LR2021_IRQ_CHIP_ERR_MASK ) != 0u )
    {
        ev |= LR2021_EVT_CHIP_ERROR;
    }
    if( ( irq & LR20XX_SYSTEM_IRQ_CRC_ERROR ) != 0u )
    {
        ev |= LR2021_EVT_CRC_ERROR;
    }
    if( ( irq & LR20XX_SYSTEM_IRQ_LEN_ERROR ) != 0u )
    {
        ev |= LR2021_EVT_LEN_ERROR;
    }

    /* Корректный пакет: RX_DONE без ошибок CRC/длины */
    if( ( ( irq & LR20XX_SYSTEM_IRQ_RX_DONE ) != 0u ) &&
        ( ( irq & ( LR20XX_SYSTEM_IRQ_CRC_ERROR | LR20XX_SYSTEM_IRQ_LEN_ERROR ) ) == 0u ) )
    {
        ev |= LR2021_EVT_RX_DONE;
    }

    /* Чип кладёт в FIFO данные даже при ошибке CRC - без очистки поток пакетов сместится */
    if( ( ev & ( LR2021_EVT_CRC_ERROR | LR2021_EVT_LEN_ERROR ) ) != 0u )
    {
        *events = ev;
        LR2021_CHECK( lr20xx_radio_fifo_clear_rx( NULL ) );
        return LR2021_OK;
    }

    *events = ev;
    return LR2021_OK;
}

lr2021_status_t lr2021_read_packet( uint8_t *buf, uint16_t buf_len, uint16_t *len )
{
    uint16_t pkt_len = 0u;

    if( ( buf == NULL ) || ( len == NULL ) )
    {
        return LR2021_ERR_PARAM;
    }
    *len = 0u;

    LR2021_CHECK( lr20xx_radio_common_get_rx_packet_length( NULL, &pkt_len ) );
    *len = pkt_len;

    if( pkt_len == 0u )
    {
        return LR2021_OK;
    }
    if( pkt_len > buf_len )
    {
        LR2021_CHECK( lr20xx_radio_fifo_clear_rx( NULL ) );
        return LR2021_ERR_BUFFER;
    }

    LR2021_CHECK( lr20xx_radio_fifo_read_rx( NULL, buf, pkt_len ) );
    return LR2021_OK;
}

lr2021_status_t lr2021_get_packet_rssi( int16_t *rssi_dbm )
{
    lr20xx_radio_fsk_packet_status_t status;

    if( rssi_dbm == NULL )
    {
        return LR2021_ERR_PARAM;
    }
    LR2021_CHECK( lr20xx_radio_fsk_get_packet_status( NULL, &status ) );
    *rssi_dbm = status.rssi_avg_in_dbm;
    return LR2021_OK;
}

lr2021_status_t lr2021_get_rx_stats( lr2021_rx_stats_t *stats )
{
    lr20xx_radio_fsk_rx_statistics_t s;

    if( stats == NULL )
    {
        return LR2021_ERR_PARAM;
    }
    LR2021_CHECK( lr20xx_radio_fsk_get_rx_statistics( NULL, &s ) );
    stats->received   = s.n_received_packets;
    stats->crc_errors = s.n_crc_errors;
    stats->len_errors = s.n_length_errors;
    stats->sync_ok    = s.n_syncword_ok;
    stats->sync_fail  = s.n_syncword_fail;
    return LR2021_OK;
}

lr2021_status_t lr2021_reset_rx_stats( void )
{
    LR2021_CHECK( lr20xx_radio_common_reset_rx_stats( NULL ) );
    return LR2021_OK;
}

lr2021_status_t lr2021_get_errors( uint16_t *errors )
{
    lr20xx_system_errors_t e = 0u;

    if( errors == NULL )
    {
        return LR2021_ERR_PARAM;
    }
    LR2021_CHECK( lr20xx_system_get_errors( NULL, &e ) );
    *errors = e;
    if( e != 0u )
    {
        LR2021_CHECK( lr20xx_system_clear_errors( NULL ) );
    }
    return LR2021_OK;
}
