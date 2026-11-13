/**
 * @file    app.c
 * @brief   Тест радиоканала FSK: передатчик шлёт пронумерованные пакеты,
 *          приёмник считает скорость полезных данных, потери и ошибки.
 *
 * Конечный автомат:
 *   INIT -> (TX) TX_SEND -> TX_WAIT -> TX_PAUSE -> TX_SEND ...
 *        -> (RX) RX_START -> RX_WAIT ...
 *   любая ошибка -> ERROR -> (пауза) -> INIT
 *
 * Формат полезной нагрузки: [0..3] номер пакета (little-endian), дальше
 * детерминированный шаблон. По номеру приёмник считает потерянные пакеты
 * (CRC их не видит), по шаблону проверяет целостность.
 *
 * Скорость полезных данных (приёмник), за окно APP_STATS_PERIOD_MS:
 *   байты всех пакетов окна, кроме первого, * 8 / (время последнего - время первого пакета).
 */

#include "app.h"

#include <stdint.h>
#include <stdio.h>

#include "board.h"

/* ------------------------------------------------------------------------- */
/* Проверки параметров и константы                                           */
/* ------------------------------------------------------------------------- */

#if ( APP_MODE != APP_MODE_TX ) && ( APP_MODE != APP_MODE_RX )
#error "APP_MODE must be APP_MODE_TX or APP_MODE_RX"
#endif

_Static_assert( ( APP_TX_PAYLOAD_LEN ) >= 4u && ( APP_TX_PAYLOAD_LEN ) <= 255u,
                "APP_TX_PAYLOAD_LEN must be 4..255" );

#define APP_PKT_MAX_LEN        255u   /* максимум, который принимает радио */
#define APP_TX_WAIT_TIMEOUT_MS 500u   /* не пришло событие TX (больше таймаута чипа) -> ошибка */
#define APP_ERROR_RETRY_MS     1000u  /* пауза перед повторным запуском после ошибки */

/* Коды bsp: -4 = буфер мал (см. board.c) */
#define APP_ERR_BUFFER         ( -4 )

typedef enum {
    APP_STATE_INIT,      /* сброс и настройка радио */
    APP_STATE_TX_SEND,   /* собрать пакет и запустить передачу */
    APP_STATE_TX_WAIT,   /* ждать окончания передачи */
    APP_STATE_TX_PAUSE,  /* сводка и ожидание начала следующего периода */
    APP_STATE_RX_START,  /* запуск непрерывного приёма */
    APP_STATE_RX_WAIT,   /* ждать пакеты */
    APP_STATE_ERROR      /* пауза перед повторным запуском */
} app_state_t;

/* ------------------------------------------------------------------------- */
/* Данные                                                                    */
/* ------------------------------------------------------------------------- */

static uint8_t s_buf[APP_PKT_MAX_LEN];

#if APP_MODE == APP_MODE_TX
static struct {
    uint32_t seq;         /* номер следующего пакета */
    uint32_t t_start_ms;  /* момент начала последней передачи */
    uint32_t t_report_ms; /* начало текущего окна */
    uint32_t t_win_us;    /* начало текущего окна, мкс */
    uint32_t w_pkts;      /* пакетов в окне */
    uint32_t w_bytes;     /* байт нагрузки в окне */
    uint32_t timeouts;    /* всего таймаутов передачи */
} s_tx;
#else
static struct {
    uint32_t t_report_ms; /* начало текущего окна */
    uint32_t w_pkts;      /* принято пакетов в окне */
    uint32_t w_bytes;     /* байты окна без первого пакета */
    uint32_t t_first_us;  /* время первого пакета окна */
    uint32_t t_last_us;   /* время последнего пакета окна */
    uint32_t w_lost;      /* потеряно по номерам, окно */
    uint32_t w_crc;       /* ошибки CRC, окно */
    uint32_t w_len;       /* ошибки длины, окно */
    uint32_t w_bad;       /* пакеты с испорченной нагрузкой, окно */
    uint32_t lost_total;  /* потеряно по номерам, всего */
    uint32_t expected;    /* ожидаемый номер пакета */
    uint8_t  seq_valid;   /* номер уже известен */
} s_rx;
#endif

/* ------------------------------------------------------------------------- */
/* Общие функции                                                             */
/* ------------------------------------------------------------------------- */

void setSeparator( char ch, uint8_t qnt )
{
    while( qnt != 0u )
    {
        printf( "%c", ch );
        qnt--;
    }
    printf( "\n" );
}

/* Байт шаблона нагрузки */
static uint8_t prv_pattern( uint32_t seq, uint16_t i )
{
    return ( uint8_t )( 0xA5u ^ ( uint8_t )( seq + i ) );
}

/* Скорость, бит/с, по байтам и времени в мкс */
static uint32_t prv_rate_bps( uint32_t bytes, uint32_t dt_us )
{
    if( dt_us == 0u )
    {
        return 0u;
    }
    return ( uint32_t )( ( ( uint64_t ) bytes * 8u * 1000000u ) / dt_us );
}

/* Наступил ли срок: прошло не менее period_ms с момента *t0 (учтено переполнение) */
static uint8_t prv_elapsed( uint32_t t0_ms, uint32_t period_ms )
{
    return ( ( uint32_t )( bsp_get_ms( ) - t0_ms ) >= period_ms ) ? 1u : 0u;
}

static app_state_t prv_fail( const char *what, int rc )
{
    printf( "%s failed, code %d\n", what, rc );
    return APP_STATE_ERROR;
}

static app_state_t prv_chip_error( void )
{
    uint16_t errors = 0u;

    ( void ) bsp_radio_get_errors( &errors );
    printf( "Chip error 0x%04X\n", ( unsigned ) errors );
    return APP_STATE_ERROR;
}

static void prv_print_banner( void )
{
    setSeparator( '=', 30 );
    printf( "Test of LR2021\n" );
    setSeparator( '~', 30 );
#if APP_MODE == APP_MODE_TX
    printf( "Transmitter\n" );
    printf( "Payload: %lu bytes, period: %lu ms%s\n", ( unsigned long ) APP_TX_PAYLOAD_LEN,
            ( unsigned long ) APP_TX_PERIOD_MS, ( APP_TX_PERIOD_MS == 0u ) ? " (back-to-back)" : "" );
#else
    printf( "Receiver\n" );
#endif
    printf( "Stats period: %lu ms\n", ( unsigned long ) APP_STATS_PERIOD_MS );
}

/* ------------------------------------------------------------------------- */
/* Инициализация                                                             */
/* ------------------------------------------------------------------------- */

static app_state_t prv_init( void )
{
    uint8_t major = 0u;
    uint8_t minor = 0u;
    int     rc;

    bsp_init( );
    bsp_delay_ms( 100u );

    setSeparator( '-', 30 );
    printf( "RadioInit...\n" );
    rc = bsp_radio_init( );
    if( rc != 0 )
    {
        return prv_fail( "RadioInit", rc );
    }

    rc = bsp_radio_get_version( &major, &minor );
    if( rc != 0 )
    {
        return prv_fail( "GetVersion", rc );
    }
    printf( "Version: major 0x%02x, minor 0x%02x\n", ( unsigned ) major, ( unsigned ) minor );
    printf( "Air bitrate: %lu bit/s\n", ( unsigned long ) bsp_radio_get_bitrate( ) );
    printf( "RadioInit done\n" );
    setSeparator( '-', 30 );

#if APP_MODE == APP_MODE_TX
    s_tx.t_report_ms = bsp_get_ms( );
    s_tx.t_win_us    = bsp_get_us( );
    s_tx.w_pkts      = 0u;
    s_tx.w_bytes     = 0u;
    return APP_STATE_TX_SEND;
#else
    return APP_STATE_RX_START;
#endif
}

/* ------------------------------------------------------------------------- */
/* Передатчик                                                                */
/* ------------------------------------------------------------------------- */

#if APP_MODE == APP_MODE_TX

static app_state_t prv_tx_send( void )
{
    int rc;

    s_buf[0] = ( uint8_t ) ( s_tx.seq );
    s_buf[1] = ( uint8_t ) ( s_tx.seq >> 8 );
    s_buf[2] = ( uint8_t ) ( s_tx.seq >> 16 );
    s_buf[3] = ( uint8_t ) ( s_tx.seq >> 24 );
    for( uint16_t i = 4u; i < APP_TX_PAYLOAD_LEN; i++ )
    {
        s_buf[i] = prv_pattern( s_tx.seq, i );
    }

    s_tx.t_start_ms = bsp_get_ms( );
    rc              = bsp_radio_tx( s_buf, APP_TX_PAYLOAD_LEN );
    if( rc != 0 )
    {
        return prv_fail( "TX start", rc );
    }
    return APP_STATE_TX_WAIT;
}

static app_state_t prv_tx_wait( void )
{
    uint32_t ev = 0u;
    int      rc = bsp_radio_poll( &ev );

    if( rc != 0 )
    {
        return prv_fail( "Radio poll", rc );
    }
    if( ( ev & BSP_RADIO_EVT_CHIP_ERROR ) != 0u )
    {
        return prv_chip_error( );
    }
    if( ( ev & BSP_RADIO_EVT_TX_DONE ) != 0u )
    {
#if APP_LOG_EVERY_PACKET
        printf( "TX #%lu done, %lu bytes, %lu ms\n", ( unsigned long ) s_tx.seq, ( unsigned long ) APP_TX_PAYLOAD_LEN,
                ( unsigned long ) ( bsp_get_ms( ) - s_tx.t_start_ms ) );
#endif
        s_tx.seq++;
        s_tx.w_pkts++;
        s_tx.w_bytes += APP_TX_PAYLOAD_LEN;
        return APP_STATE_TX_PAUSE;
    }
    if( ( ev & BSP_RADIO_EVT_TIMEOUT ) != 0u )
    {
        s_tx.timeouts++;
        printf( "TX #%lu timeout\n", ( unsigned long ) s_tx.seq );
        s_tx.seq++;
        return APP_STATE_TX_PAUSE;
    }
    if( prv_elapsed( s_tx.t_start_ms, APP_TX_WAIT_TIMEOUT_MS ) != 0u )
    {
        printf( "TX #%lu: no event from radio\n", ( unsigned long ) s_tx.seq );
        return APP_STATE_ERROR;
    }
    return APP_STATE_TX_WAIT;
}

static void prv_tx_report( void )
{
    const uint32_t now_us = bsp_get_us( );
    const uint32_t rate   = prv_rate_bps( s_tx.w_bytes, ( uint32_t )( now_us - s_tx.t_win_us ) );

    printf( "TX pkt=%lu %lu bit/s timeouts=%lu\n", ( unsigned long ) s_tx.w_pkts, ( unsigned long ) rate,
            ( unsigned long ) s_tx.timeouts );

    s_tx.w_pkts      = 0u;
    s_tx.w_bytes     = 0u;
    s_tx.t_win_us    = now_us;
    s_tx.t_report_ms = bsp_get_ms( );
}

static app_state_t prv_tx_pause( void )
{
    if( prv_elapsed( s_tx.t_report_ms, APP_STATS_PERIOD_MS ) != 0u )
    {
        prv_tx_report( ); /* печать блокирует цикл, поэтому только в паузе, не в полёте пакета */
    }
    if( prv_elapsed( s_tx.t_start_ms, APP_TX_PERIOD_MS ) != 0u )
    {
        return APP_STATE_TX_SEND;
    }
    return APP_STATE_TX_PAUSE;
}

#endif /* APP_MODE_TX */

/* ------------------------------------------------------------------------- */
/* Приёмник                                                                  */
/* ------------------------------------------------------------------------- */

#if APP_MODE == APP_MODE_RX

static void prv_rx_window_reset( void )
{
    s_rx.w_pkts      = 0u;
    s_rx.w_bytes     = 0u;
    s_rx.w_lost      = 0u;
    s_rx.w_crc       = 0u;
    s_rx.w_len       = 0u;
    s_rx.w_bad       = 0u;
    s_rx.t_report_ms = bsp_get_ms( );
}

/* Учёт принятого корректного пакета (CRC прошёл) */
static void prv_rx_on_packet( const uint8_t *p, uint16_t len, uint32_t t_us )
{
    if( s_rx.w_pkts == 0u )
    {
        s_rx.t_first_us = t_us;
    }
    else
    {
        s_rx.w_bytes += len; /* первый пакет окна не считается: интервал отсчитывается от его прихода */
    }
    s_rx.t_last_us = t_us;
    s_rx.w_pkts++;

    if( len < 4u )
    {
        s_rx.w_bad++;
        return;
    }

    const uint32_t seq = ( uint32_t ) p[0] | ( ( uint32_t ) p[1] << 8 ) | ( ( uint32_t ) p[2] << 16 ) |
                         ( ( uint32_t ) p[3] << 24 );

    if( s_rx.seq_valid != 0u )
    {
        const uint32_t diff = seq - s_rx.expected;

        if( diff < 0x80000000u )
        {
            s_rx.w_lost += diff;
            s_rx.lost_total += diff;
        }
        /* иначе номер меньше ожидаемого: передатчик перезапустился, просто синхронизируемся */
    }
    s_rx.seq_valid = 1u;
    s_rx.expected  = seq + 1u;

    for( uint16_t i = 4u; i < len; i++ )
    {
        if( p[i] != prv_pattern( seq, i ) )
        {
            s_rx.w_bad++;
            break;
        }
    }

#if APP_LOG_EVERY_PACKET
    printf( "RX #%lu %u bytes\n", ( unsigned long ) seq, ( unsigned ) len );
#endif
}

static void prv_rx_report( void )
{
    int16_t        rssi = 0;
    const uint32_t rate = ( s_rx.w_pkts > 1u )
                              ? prv_rate_bps( s_rx.w_bytes, ( uint32_t )( s_rx.t_last_us - s_rx.t_first_us ) )
                              : 0u;

    if( s_rx.w_pkts > 0u )
    {
        if( bsp_radio_get_rssi( &rssi ) != 0 )
        {
            rssi = 0;
        }
    }

    printf( "RX pkt=%lu %lu bit/s lost=%lu(%lu) crc=%lu len=%lu bad=%lu rssi=%d\n", ( unsigned long ) s_rx.w_pkts,
            ( unsigned long ) rate, ( unsigned long ) s_rx.w_lost, ( unsigned long ) s_rx.lost_total,
            ( unsigned long ) s_rx.w_crc, ( unsigned long ) s_rx.w_len, ( unsigned long ) s_rx.w_bad,
            ( int ) rssi );

#if APP_LOG_CHIP_STATS
    {
        uint16_t c_rx = 0u;
        uint16_t c_crc = 0u;
        uint16_t c_len = 0u;

        if( bsp_radio_get_rx_stats( &c_rx, &c_crc, &c_len ) == 0 )
        {
            printf( "  chip: rx=%u crc=%u len=%u\n", ( unsigned ) c_rx, ( unsigned ) c_crc, ( unsigned ) c_len );
        }
    }
#endif

    prv_rx_window_reset( );
}

static app_state_t prv_rx_start( void )
{
    const int rc = bsp_radio_rx_start( );

    if( rc != 0 )
    {
        return prv_fail( "RX start", rc );
    }
    ( void ) bsp_radio_reset_rx_stats( );
    s_rx.seq_valid = 0u;
    s_rx.lost_total = 0u;
    prv_rx_window_reset( );
    printf( "RX started\n" );
    return APP_STATE_RX_WAIT;
}

static app_state_t prv_rx_wait( void )
{
    uint32_t ev = 0u;
    int      rc = bsp_radio_poll( &ev );

    if( rc != 0 )
    {
        return prv_fail( "Radio poll", rc );
    }

    if( ev != 0u )
    {
        const uint32_t t_us = bsp_get_us( ); /* момент обработки события, ближайший к приходу пакета */

        if( ( ev & BSP_RADIO_EVT_CHIP_ERROR ) != 0u )
        {
            return prv_chip_error( );
        }
        if( ( ev & BSP_RADIO_EVT_RX_DONE ) != 0u )
        {
            uint16_t len = 0u;

            rc = bsp_radio_read_packet( s_buf, sizeof( s_buf ), &len );
            if( rc == APP_ERR_BUFFER )
            {
                s_rx.w_len++;
            }
            else if( rc != 0 )
            {
                return prv_fail( "Read packet", rc );
            }
            else
            {
                prv_rx_on_packet( s_buf, len, t_us );
            }
        }
        if( ( ev & BSP_RADIO_EVT_CRC_ERROR ) != 0u )
        {
            s_rx.w_crc++;
        }
        if( ( ev & BSP_RADIO_EVT_LEN_ERROR ) != 0u )
        {
            s_rx.w_len++;
        }
        if( ( ev & BSP_RADIO_EVT_TIMEOUT ) != 0u )
        {
            printf( "RX timeout, restart\n" );
            return APP_STATE_RX_START;
        }
    }

    if( prv_elapsed( s_rx.t_report_ms, APP_STATS_PERIOD_MS ) != 0u )
    {
        prv_rx_report( );
    }
    return APP_STATE_RX_WAIT;
}

#endif /* APP_MODE_RX */

/* ------------------------------------------------------------------------- */
/* Точка входа                                                               */
/* ------------------------------------------------------------------------- */

void app( void )
{
    app_state_t state = APP_STATE_INIT;

    prv_print_banner( );

    while( 1 )
    {
        switch( state )
        {
        case APP_STATE_INIT:
            state = prv_init( );
            break;

#if APP_MODE == APP_MODE_TX
        case APP_STATE_TX_SEND:
            state = prv_tx_send( );
            break;

        case APP_STATE_TX_WAIT:
            state = prv_tx_wait( );
            break;

        case APP_STATE_TX_PAUSE:
            state = prv_tx_pause( );
            break;
#else
        case APP_STATE_RX_START:
            state = prv_rx_start( );
            break;

        case APP_STATE_RX_WAIT:
            state = prv_rx_wait( );
            break;
#endif

        case APP_STATE_ERROR:
            printf( "Restart in %lu ms\n", ( unsigned long ) APP_ERROR_RETRY_MS );
            bsp_delay_ms( APP_ERROR_RETRY_MS );
            state = APP_STATE_INIT;
            break;

        default:
            state = APP_STATE_ERROR;
            break;
        }
    }
}
