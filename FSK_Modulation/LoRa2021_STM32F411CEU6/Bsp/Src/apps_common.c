/*!
 * @file      apps_common.c
 *
 * @brief     Common functions shared by the examples
 *
 * @copyright
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "apps_common.h"
#include "lr20xx_driver_version.h"
#include "lr20xx_workaround.h"
#include "lr20xx_radio_lora.h"
#include "lr20xx_radio_fsk.h"
#include "lr20xx_radio_fifo.h"
#include "lr20xx_radio_flrc.h"
#include "lr20xx_regmem.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#ifdef LR2021
smtc_shield_lr20xx_t shield = SMTC_SHIELD_LR2021_INSTANTIATE;
#endif

static lr20xx_radio_lora_mod_params_t lora_mod_params = {
    .sf = LORA_SPREADING_FACTOR, .bw = LORA_BANDWIDTH, .cr = LORA_CODING_RATE, .ppm = LR20XX_RADIO_LORA_NO_PPM
};

static const lr20xx_radio_lora_pkt_params_t lora_pkt_params = {
    .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
    .pkt_mode             = LORA_PKT_LEN_MODE,
    .pld_len_in_bytes     = ( uint8_t ) PAYLOAD_LENGTH,
    .crc                  = LORA_CRC,
    .iq                   = LORA_IQ,
};

static const lr20xx_radio_fsk_mod_params_t gfsk_mod_params = {
    .br          = FSK_BITRATE,
    .pulse_shape = FSK_PULSE_SHAPE,
    .bw          = FSK_BANDWIDTH,
    .fdev_in_hz  = FSK_FDEV,
};

static const lr20xx_radio_fsk_pkt_params_t fsk_pkt_params = {
    .pbl_length_in_bit = FSK_PREAMBLE_LENGTH,
    .preamble_detector = FSK_PREAMBLE_DETECTOR,
    .address_filtering = FSK_ADDRESS_FILTERING,
    .header_mode       = FSK_HEADER_TYPE,
    .payload_length    = PAYLOAD_LENGTH,
    .crc               = FSK_CRC_TYPE,
    .whitening         = FSK_DC_FREE,
};

static const lr20xx_radio_flrc_mod_params_t flrc_mod_params = { .br_bw = FLRC_BANDWIDTH,
                                                                .cr    = FLRC_CODING_RATE,
                                                                .shape = FLRC_PULSE_SHAPE };

static const lr20xx_radio_flrc_pkt_params_t flrc_pkt_params = { .preamble_len     = FLRC_PREAMBLE_LEN,
                                                                .sync_word_len    = FLRC_SYNCWORD_LEN,
                                                                .match_sync_word  = FLRC_RX_SYNCWORD_MATCH,
                                                                .header_type      = FLRC_PKT_LEN_MODE,
                                                                .pld_len_in_bytes = PAYLOAD_LENGTH,
                                                                .crc_type         = FLRC_CRC_TYPE };

/*static const lr20xx_radio_mod_params_bpsk_t bpsk_mod_params = {
    .br_in_bps   = BPSK_BITRATE_IN_BPS,
    .pulse_shape = LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_0_7,
};

static lr20xx_radio_pkt_params_bpsk_t bpsk_pkt_params = {
    .pld_len_in_bytes = 0,  // Will be initialized in radio init
    .ramp_up_delay    = 0,
    .ramp_down_delay  = 0,
    .pld_len_in_bits  = 0,  // Will be initialized in radio init
};*/

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr20xx_hal_context_t context;

static volatile bool irq_fired = false;



/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Print the common configuration on the debug interface
 */
void print_common_configuration( void );

/*!
 * @brief Print the LoRa configuration on the debug interface
 */
void print_lora_configuration( void );

/*!
 * @brief Print the GFSK configuration on the debug interface
 */
void print_gfsk_configuration( void );

/*!
 * @brief Print the LR11XX driver version
 */
static void print_driver_version( void );

void radio_on_dio_irq( void* context );
void on_tx_done( void ) __attribute__( ( weak ) );
void on_rx_done( void ) __attribute__( ( weak ) );
void on_rx_tx_timeout( void ) __attribute__( ( weak ) );
void on_rx_fifo_threshold( void ) __attribute__( ( weak ) );
void on_tx_fifo_threshold( void ) __attribute__( ( weak ) );
void on_preamble_detected( void ) __attribute__( ( weak ) );
void on_syncword_header_valid( void ) __attribute__( ( weak ) );
void on_lora_header_error( void ) __attribute__( ( weak ) );
void on_rx_len_error( void ) __attribute__( ( weak ) );
void on_rx_crc_error( void ) __attribute__( ( weak ) );
void on_cad_done_undetected( void ) __attribute__( ( weak ) );
void on_cad_done_detected( void ) __attribute__( ( weak ) );
void on_rx_timestamp( void ) __attribute__( ( weak ) );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */



void apps_common_lr20xx_set_dio_irq_params( const void* context, const apps_common_lr20xx_irq_cfg_t* irq_cfg,
                                            const uint8_t irq_cfg_len )
{
    lr20xx_system_clear_irq_status( context, LR20XX_SYSTEM_IRQ_ALL_MASK );

    for( uint8_t i = 0; i < irq_cfg_len; i++ )
    {
        lr20xx_system_dio_t      dio          = irq_cfg->dio;
        lr20xx_system_dio_func_t dio_function = smtc_shield_lr20xx_get_dio_function( &shield, dio );

        if( dio_function != LR20XX_SYSTEM_DIO_FUNC_IRQ )
        {
            continue;
        }

        lr20xx_system_set_dio_irq_cfg( context, dio, irq_cfg->irq_mask );

        irq_cfg += sizeof( apps_common_lr20xx_irq_cfg_t );
    }
}

void apps_common_lr20xx_system_init( const lr20xx_hal_context_t* context )
{
    ASSERT_LR20XX_RC( lr20xx_system_reset( ( void* ) context ) );

    // Configure the regulator
    const lr20xx_system_reg_mode_t regulator = smtc_shield_lr20xx_get_reg_mode( &shield );
    ASSERT_LR20XX_RC( lr20xx_system_set_reg_mode( ( void* ) context, regulator ) );

    const smtc_shield_lr20xx_xosc_cfg_t* tcxo_cfg = smtc_shield_lr20xx_get_xosc_cfg( &shield );
    if( tcxo_cfg->has_tcxo == true )
    {
        ASSERT_LR20XX_RC( lr20xx_system_set_tcxo_mode( context, tcxo_cfg->supply, tcxo_cfg->startup_time_in_tick ) );
    }

    const lr20xx_system_lfclk_cfg_t lfclk_cfg = smtc_shield_lr20xx_get_lfclk_cfg( &shield );
    ASSERT_LR20XX_RC( lr20xx_system_cfg_lfclk( context, lfclk_cfg ) );

    ASSERT_LR20XX_RC( lr20xx_system_clear_errors( context ) );

    ASSERT_LR20XX_RC( lr20xx_workaround_load_and_enable_patch( context ) );

    ASSERT_LR20XX_RC( lr20xx_system_clear_errors( context ) );
    ASSERT_LR20XX_RC( lr20xx_system_clear_irq_status( context, LR20XX_SYSTEM_IRQ_ALL_MASK ) );

    uint8_t dio_count = lr20xx_system_dio_get_count( );
    for( uint8_t dio_nth = 0; dio_nth < dio_count; dio_nth++ )
    {
        lr20xx_system_dio_t dio;
        if( !lr20xx_system_dio_get_nth( dio_nth, &dio ) )
        {
            return;
        }

        lr20xx_system_dio_func_t  dio_function = smtc_shield_lr20xx_get_dio_function( &shield, dio );
        lr20xx_system_dio_drive_t dio_drive    = smtc_shield_lr20xx_get_dio_sleep_drive( &shield, dio );

        lr20xx_system_set_dio_function( context, dio, dio_function, dio_drive );

        if( dio_function == LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH )
        {
            lr20xx_system_dio_rf_switch_cfg_t rf_switch_cfg = smtc_shield_lr20xx_get_rf_switch_cfg( &shield, dio );
            lr20xx_system_set_dio_rf_switch_cfg( context, dio, rf_switch_cfg );
        }

        if( dio_function == LR20XX_SYSTEM_DIO_FUNC_HF_CLK_OUT )
        {
            // unsupported for now

            /*lr20xx_system_hf_clk_scaling_t hf_clk_scaling;
            ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg( context, &hf_clk_scaling );
            status = lr20xx_system_cfg_clk_output( context, hf_clk_scaling );
            if( status != RAL_STATUS_OK )
            {
                return status;
            }*/
        }
    }
}

void apps_common_lr20xx_fetch_and_print_version( const lr20xx_hal_context_t* context )
{
    lr20xx_system_version_t version;

    apps_common_lr20xx_fetch_version( ( void* ) context, &version );
    apps_common_lr20xx_print_version( &version );
}

void apps_common_lr20xx_fetch_version( const lr20xx_hal_context_t* context, lr20xx_system_version_t* version )
{
    ASSERT_LR20XX_RC( lr20xx_system_get_version( ( void* ) context, version ) );
}

void apps_common_lr20xx_print_version( const lr20xx_system_version_t* version )
{
    HAL_DBG_TRACE_INFO( "LR20xx information:\n" );
    HAL_DBG_TRACE_INFO( "  - Firmware mayor = 0x%04X\n", version->major );
    HAL_DBG_TRACE_INFO( "  - Firmware minor = 0x%02X\n", version->minor );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void apps_common_print_sdk_driver_version( void )
{
    common_version_print( );
    print_driver_version( );
    HAL_DBG_TRACE_INFO( "\n" );
}

void apps_common_lr20xx_radio_init( const void* context )
{
    const smtc_shield_lr20xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr20xx_get_pa_pwr_cfg( &shield, RF_FREQ_IN_HZ, TX_OUTPUT_POWER_HALF_DBM_STEPS );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    // print_common_configuration( );

    ASSERT_LR20XX_RC( lr20xx_radio_common_set_pkt_type( context, PACKET_TYPE ) );
    ASSERT_LR20XX_RC( lr20xx_radio_common_set_rf_freq( context, RF_FREQ_IN_HZ ) );

    lr20xx_radio_common_rx_path_t rx_path = pa_pwr_cfg->pa_config.pa_sel == LR20XX_RADIO_COMMON_PA_SEL_HF
                                                ? LR20XX_RADIO_COMMON_RX_PATH_HF
                                                : LR20XX_RADIO_COMMON_RX_PATH_LF;
    lr20xx_radio_common_front_end_calibration_value_t calibration = { .frequency_in_hertz = RF_FREQ_IN_HZ,
                                                                      .rx_path            = rx_path };
    ASSERT_LR20XX_RC( lr20xx_radio_common_calibrate_front_end_helper( context, &calibration, 1 ) );
    ASSERT_LR20XX_RC( lr20xx_radio_common_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );
    ASSERT_LR20XX_RC( lr20xx_radio_common_set_tx_params( context, TX_OUTPUT_POWER_HALF_DBM_STEPS, PA_RAMP_TIME ) );
    ASSERT_LR20XX_RC( lr20xx_radio_common_set_rx_tx_fallback_mode( context, FALLBACK_MODE ) );
    ASSERT_LR20XX_RC( lr20xx_radio_common_set_rx_path( context, rx_path, ENABLE_RX_BOOST_MODE ) );

    switch( PACKET_TYPE )
    {
    case LR20XX_RADIO_COMMON_PKT_TYPE_LORA:
        // print_lora_configuration( );

        lora_mod_params.ppm = apps_common_compute_lora_ppm( LORA_SPREADING_FACTOR, LORA_BANDWIDTH );
        ASSERT_LR20XX_RC( lr20xx_radio_lora_set_modulation_params( context, &lora_mod_params ) );
        ASSERT_LR20XX_RC( lr20xx_radio_lora_set_packet_params( context, &lora_pkt_params ) );
        ASSERT_LR20XX_RC( lr20xx_radio_lora_set_syncword( context, LORA_SYNCWORD ) );
        break;
    case LR20XX_RADIO_COMMON_PKT_TYPE_FSK:
        // print_gfsk_configuration( );

        ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_modulation_params( context, &gfsk_mod_params ) );
        ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_packet_params( context, &fsk_pkt_params ) );
        ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_syncword( context, gfsk_sync_word, FSK_SYNCWORD_LENGTH,
                                                         LR20XX_RADIO_FSK_SYNCWORD_MSBF ) );

        if( FSK_DC_FREE != LR20XX_RADIO_FSK_WHITENING_OFF )
        {
            ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_whitening_params(
                context, LR20XX_RADIO_FSK_WHITENING_COMPATIBILITY_SX126X_LR11XX, FSK_WHITENING_SEED ) );
        }

        if( FSK_CRC_TYPE != LR20XX_RADIO_FSK_CRC_OFF )
        {
            ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_crc_params( context, FSK_CRC_POLYNOMIAL, FSK_CRC_SEED ) );
        }

        if( FSK_ADDRESS_FILTERING != LR20XX_RADIO_FSK_ADDRESS_FILTERING_DISABLED )
        {
            ASSERT_LR20XX_RC( lr20xx_radio_fsk_set_addresses( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS ) );
        }
        break;
    case LR20XX_RADIO_COMMON_PKT_TYPE_FLRC:
        ASSERT_LR20XX_RC( lr20xx_radio_flrc_set_modulation_params( context, &flrc_mod_params ) );
        ASSERT_LR20XX_RC( lr20xx_radio_flrc_set_pkt_params( context, &flrc_pkt_params ) );
        break;
    default:
        break;
    }
}

lr20xx_status_t apps_common_lr20xx_radio_get_last_cmd_status( const void* context )
{
    lr20xx_system_stat1_t    stat1;
    lr20xx_system_stat2_t    stat2;
    lr20xx_system_irq_mask_t irq_status;
    lr20xx_system_get_status( context, &stat1, &stat2, &irq_status );

    if( stat1.command_status >= LR20XX_SYSTEM_CMD_STATUS_OK )
    {
        return LR20XX_STATUS_OK;
    }

    return LR20XX_STATUS_ERROR;
}

void apps_common_lr20xx_read_length_from_header( void* context, uint16_t* length )
{
    uint32_t reg_32 = 0;
    lr20xx_regmem_read_regmem32( context, 0xF30800 + 0x74, &reg_32, 1 );
    apps_utilities_reverse_bits( &reg_32 );
    reg_32 >>= 16;
    *length = ( uint16_t ) reg_32;
}

// unsupported for now
/*void apps_common_lr11xx_radio_dbpsk_init( const void* context, const uint8_t payload_len )
{
    const smtc_shield_lr11xx_pa_pwr_cfg_t* pa_pwr_cfg =
        smtc_shield_lr11xx_get_pa_pwr_cfg( &shield, SIGFOX_UPLINK_RF_FREQ_IN_HZ, SIGFOX_TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        HAL_DBG_TRACE_ERROR( "Invalid target frequency or power level\n" );
        while( true )
        {
        }
    }

    HAL_DBG_TRACE_INFO( "Sigfox parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( LR11XX_RADIO_PKT_TYPE_BPSK ) );
    HAL_DBG_TRACE_INFO( "   RF frequency  = %u Hz\n", SIGFOX_UPLINK_RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_INFO( "   Output power  = %i dBm\n", SIGFOX_TX_OUTPUT_POWER_DBM );

    ASSERT_LR11XX_RC( lr11xx_radio_set_pkt_type( context, LR11XX_RADIO_PKT_TYPE_BPSK ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rf_freq( context, SIGFOX_UPLINK_RF_FREQ_IN_HZ ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_rssi_calibration(
        context, smtc_shield_lr11xx_get_rssi_calibration_table( &shield, SIGFOX_UPLINK_RF_FREQ_IN_HZ ) ) );
    ASSERT_LR11XX_RC( lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) ) );

    ASSERT_LR11XX_RC( lr11xx_radio_set_bpsk_mod_params( context, &bpsk_mod_params ) );

    bpsk_pkt_params.pld_len_in_bytes = smtc_dbpsk_get_pld_len_in_bytes( payload_len << 3 );
    bpsk_pkt_params.pld_len_in_bits  = smtc_dbpsk_get_pld_len_in_bits( payload_len << 3 );

    if( BPSK_BITRATE_IN_BPS == 100 )
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_100_BPS;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_100_BPS;
    }
    else if( BPSK_BITRATE_IN_BPS == 600 )
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_600_BPS;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_600_BPS;
    }
    else
    {
        bpsk_pkt_params.ramp_up_delay   = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_UP_TIME_DEFAULT;
        bpsk_pkt_params.ramp_down_delay = LR11XX_RADIO_SIGFOX_DBPSK_RAMP_DOWN_TIME_DEFAULT;
    }

    ASSERT_LR11XX_RC( lr11xx_radio_set_bpsk_pkt_params( context, &bpsk_pkt_params ) );
}*/

void apps_common_lr20xx_receive( const void* context, uint8_t* buffer, uint16_t buffer_length, uint16_t* size )
{
    lr20xx_radio_common_get_rx_packet_length( context, size );

    if( *size > buffer_length )
    {
        HAL_DBG_TRACE_ERROR( "Received payload (size: %d) is bigger than the buffer (size: %d)!\n", *size,
                             buffer_length );

        lr20xx_radio_flrc_rx_stats_t rx_stats;
        lr20xx_radio_flrc_get_rx_stats( context, &rx_stats );
        HAL_DBG_TRACE_INFO( "  - RX packets = %u\n", rx_stats.received_packets );
        HAL_DBG_TRACE_INFO( "  - CRC errors = %u\n", rx_stats.crc_errors );
        HAL_DBG_TRACE_INFO( "  - Length errors = %u\n", rx_stats.length_errors );
        return;
    }
    lr20xx_radio_fifo_read_rx( context, buffer, *size );

    HAL_DBG_TRACE_ARRAY( "Packet content", buffer, *size );

    HAL_DBG_TRACE_INFO( "Packet status:\n" );

    switch( PACKET_TYPE )
    {
    case LR20XX_RADIO_COMMON_PKT_TYPE_LORA:
    {
        lr20xx_radio_lora_packet_status_t pkt_status_lora;
        lr20xx_radio_lora_get_packet_status( context, &pkt_status_lora );
        HAL_DBG_TRACE_INFO( "  - RSSI packet = %i dBm\n", pkt_status_lora.rssi_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - Signal RSSI packet = %i dBm\n", pkt_status_lora.rssi_signal_pkt_in_dbm );
        HAL_DBG_TRACE_INFO( "  - SNR packet = %i dB\n", pkt_status_lora.snr_pkt_in_db );
        break;
    }
    case LR20XX_RADIO_COMMON_PKT_TYPE_FSK:
    {
        lr20xx_radio_fsk_packet_status_t pkt_status_fsk;
        lr20xx_radio_fsk_get_packet_status( context, &pkt_status_fsk );
        HAL_DBG_TRACE_INFO( "  - RSSI average = %i dBm\n", pkt_status_fsk.rssi_avg_in_dbm );
        HAL_DBG_TRACE_INFO( "  - RSSI sync = %i dBm\n", pkt_status_fsk.rssi_sync_in_dbm );
        break;
    }
    case LR20XX_RADIO_COMMON_PKT_TYPE_FLRC:
    {
        lr20xx_radio_flrc_pkt_status_t pkt_status_flrc;
        lr20xx_radio_flrc_get_pkt_status( context, &pkt_status_flrc );
        HAL_DBG_TRACE_INFO( "  - RSSI average = %i dBm\n", pkt_status_flrc.rssi_avg_in_dbm );
        lr20xx_radio_flrc_rx_stats_t rx_stats;
        lr20xx_radio_flrc_get_rx_stats( context, &rx_stats );
        HAL_DBG_TRACE_INFO( "  - RX packets = %u\n", rx_stats.received_packets );
        HAL_DBG_TRACE_INFO( "  - CRC errors = %u\n", rx_stats.crc_errors );
        HAL_DBG_TRACE_INFO( "  - Length errors = %u\n", rx_stats.length_errors );
        break;
    }
    default:
        break;
    }
}

void apps_common_lr20xx_irq_process( const void* context, lr20xx_system_irq_mask_t irq_filter_mask, bool log_enable )
{
    if( irq_fired == true )
    {
        irq_fired = false;

        lr20xx_system_irq_mask_t irq_regs;
        lr20xx_system_get_and_clear_irq_status( context, &irq_regs );

        if( log_enable )
        {
            HAL_DBG_TRACE_INFO( "Interrupt flags = 0x%08X\n", irq_regs );
        }

        irq_regs &= irq_filter_mask;

        if( log_enable )
        {
            HAL_DBG_TRACE_INFO( "Interrupt flags (after filtering) = 0x%08X\n", irq_regs );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_TX_DONE ) == LR20XX_SYSTEM_IRQ_TX_DONE )
        {
            HAL_DBG_TRACE_INFO( "Tx done\n" );
            on_tx_done( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED ) == LR20XX_SYSTEM_IRQ_PREAMBLE_DETECTED )
        {
            HAL_DBG_TRACE_INFO( "Preamble detected\n" );
            on_preamble_detected( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR ) == LR20XX_SYSTEM_IRQ_LORA_HEADER_ERROR )
        {
            HAL_DBG_TRACE_ERROR( "LoRa header error\n" );
            on_lora_header_error( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_FIFO_TX ) == LR20XX_SYSTEM_IRQ_FIFO_TX )
        {
            on_tx_fifo_threshold( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_FIFO_RX ) == LR20XX_SYSTEM_IRQ_FIFO_RX )
        {
            on_rx_fifo_threshold( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) == LR20XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID )
        {
            HAL_DBG_TRACE_INFO( "Syncword or header valid\n" );
            on_syncword_header_valid( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_RX_DONE ) == LR20XX_SYSTEM_IRQ_RX_DONE )
        {
            if( ( irq_regs & LR20XX_SYSTEM_IRQ_CRC_ERROR ) == LR20XX_SYSTEM_IRQ_CRC_ERROR )
            {
                HAL_DBG_TRACE_ERROR( "CRC error\n" );
                on_rx_crc_error( );
            }
            else if( ( irq_regs & LR20XX_SYSTEM_IRQ_LEN_ERROR ) == LR20XX_SYSTEM_IRQ_LEN_ERROR )
            {
                HAL_DBG_TRACE_ERROR( "RX length error\n" );
                on_rx_len_error( );
            }
            else
            {
                HAL_DBG_TRACE_INFO( "Rx done\n" );
                on_rx_done( );
            }
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_CAD_DONE ) == LR20XX_SYSTEM_IRQ_CAD_DONE )
        {
            HAL_DBG_TRACE_INFO( "CAD done\n" );
            if( ( irq_regs & LR20XX_SYSTEM_IRQ_CAD_DETECTED ) == LR20XX_SYSTEM_IRQ_CAD_DETECTED )
            {
                HAL_DBG_TRACE_INFO( "Channel activity detected\n" );
                on_cad_done_detected( );
            }
            else
            {
                HAL_DBG_TRACE_INFO( "No channel activity detected\n" );
                on_cad_done_undetected( );
            }
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_TIMEOUT ) == LR20XX_SYSTEM_IRQ_TIMEOUT )
        {
            on_rx_tx_timeout( );
        }

        if( ( irq_regs & LR20XX_SYSTEM_IRQ_RX_TIMESTAMP ) == LR20XX_SYSTEM_IRQ_RX_TIMESTAMP )
        {
            on_rx_timestamp( );
        }

        HAL_DBG_TRACE_PRINTF( "\n" );
    }
}

void apps_common_lr20xx_handle_pre_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr20xx_handle_post_tx( void )
{
    if( shield_pinout->led_tx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_tx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr20xx_handle_pre_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr20xx_handle_post_rx( void )
{
    if( shield_pinout->led_rx != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_rx.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr20xx_handle_pre_gnss_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }

    if( shield_pinout->lna != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( lna_ctrl.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr20xx_handle_post_gnss_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }

    if( shield_pinout->lna != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( lna_ctrl.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

void apps_common_lr20xx_handle_pre_wifi_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_HIGH );
    }
}

void apps_common_lr20xx_handle_post_wifi_scan( void )
{
    if( shield_pinout->led_scan != SMTC_SHIELD_PINOUT_NONE )
    {
        smtc_hal_mcu_gpio_set_state( led_scan.inst, SMTC_HAL_MCU_GPIO_STATE_LOW );
    }
}

uint32_t get_time_on_air_in_ms( void )
{
    switch( PACKET_TYPE )
    {
    case LR20XX_RADIO_COMMON_PKT_TYPE_LORA:
    {
        return lr20xx_radio_lora_get_time_on_air_in_ms( &lora_pkt_params, &lora_mod_params );
    }
    case LR20XX_RADIO_COMMON_PKT_TYPE_FSK:
    {
        return lr20xx_radio_fsk_get_time_on_air_in_ms( &fsk_pkt_params, &gfsk_mod_params, FSK_SYNCWORD_LENGTH );
    }
    default:
    {
        return 0;
    }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*void print_common_configuration( void )
{
    HAL_DBG_TRACE_INFO( "Common parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Packet type   = %s\n", lr11xx_radio_pkt_type_to_str( PACKET_TYPE ) );
    HAL_DBG_TRACE_INFO( "   RF frequency  = %u Hz\n", RF_FREQ_IN_HZ );
    HAL_DBG_TRACE_INFO( "   Output power  = %i dBm\n", TX_OUTPUT_POWER_DBM );
    HAL_DBG_TRACE_INFO( "   Fallback mode = %s\n", lr11xx_radio_fallback_modes_to_str( FALLBACK_MODE ) );
    HAL_DBG_TRACE_INFO( ( ENABLE_RX_BOOST_MODE == true ) ? "   Rx boost activated\n" : "   Rx boost deactivated\n");
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void print_lora_configuration( void )
{
    HAL_DBG_TRACE_INFO( "LoRa modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Spreading factor = %s\n", lr11xx_radio_lora_sf_to_str( LORA_SPREADING_FACTOR ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth        = %s\n", lr11xx_radio_lora_bw_to_str( LORA_BANDWIDTH ) );
    HAL_DBG_TRACE_INFO( "   Coding rate      = %s\n", lr11xx_radio_lora_cr_to_str( LORA_CODING_RATE ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length = %d symbol(s)\n", LORA_PREAMBLE_LENGTH );
    HAL_DBG_TRACE_INFO( "   Header mode     = %s\n", lr11xx_radio_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) );
    HAL_DBG_TRACE_INFO( "   Payload length  = %d byte(s)\n", PAYLOAD_LENGTH );
    HAL_DBG_TRACE_INFO( "   CRC mode        = %s\n", lr11xx_radio_lora_crc_to_str( LORA_CRC ) );
    HAL_DBG_TRACE_INFO( "   IQ              = %s\n", lr11xx_radio_lora_iq_to_str( LORA_IQ ) );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "LoRa syncword = 0x%02X\n", LORA_SYNCWORD );
    HAL_DBG_TRACE_PRINTF( "\n" );
}

void print_gfsk_configuration( void )
{
    HAL_DBG_TRACE_INFO( "GFSK modulation parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Bitrate             = %u bps\n", FSK_BITRATE );
    HAL_DBG_TRACE_INFO( "   Pulse shape         = %s\n", lr11xx_radio_gfsk_pulse_shape_to_str( FSK_PULSE_SHAPE ) );
    HAL_DBG_TRACE_INFO( "   Bandwidth           = %s\n", lr11xx_radio_gfsk_bw_to_str( FSK_BANDWIDTH ) );
    HAL_DBG_TRACE_INFO( "   Frequency deviation = %u Hz\n", FSK_FDEV );
    HAL_DBG_TRACE_PRINTF( "\n" );

    HAL_DBG_TRACE_INFO( "GFSK packet parameters:\n" );
    HAL_DBG_TRACE_INFO( "   Preamble length   = %d bit(s)\n", FSK_PREAMBLE_LENGTH );
    HAL_DBG_TRACE_INFO( "   Preamble detector = %s\n",
                        lr11xx_radio_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) );
    HAL_DBG_TRACE_INFO( "   Syncword length   = %d bit(s)\n", FSK_SYNCWORD_LENGTH );
    HAL_DBG_TRACE_INFO( "   Address filtering = %s\n",
                        lr11xx_radio_gfsk_address_filtering_to_str( FSK_ADDRESS_FILTERING ) );
    if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
    {
        HAL_DBG_TRACE_INFO( "     (Node address      = 0x%02X)\n", FSK_NODE_ADDRESS );
        if( FSK_ADDRESS_FILTERING == LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES )
        {
            HAL_DBG_TRACE_INFO( "     (Broadcast address = 0x%02X)\n", FSK_BROADCAST_ADDRESS );
        }
    }
    HAL_DBG_TRACE_INFO( "   Header mode       = %s\n", lr11xx_radio_gfsk_pkt_len_modes_to_str( FSK_HEADER_TYPE ) );
    HAL_DBG_TRACE_INFO( "   Payload length    = %d byte(s)\n", PAYLOAD_LENGTH );
    HAL_DBG_TRACE_INFO( "   CRC mode          = %s\n", lr11xx_radio_gfsk_crc_type_to_str( FSK_CRC_TYPE ) );
    if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (CRC seed       = 0x%08X)\n", FSK_CRC_SEED );
        HAL_DBG_TRACE_INFO( "     (CRC polynomial = 0x%08X)\n", FSK_CRC_POLYNOMIAL );
    }
    HAL_DBG_TRACE_INFO( "   DC free           = %s\n", lr11xx_radio_gfsk_dc_free_to_str( FSK_DC_FREE ) );
    if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
    {
        HAL_DBG_TRACE_INFO( "     (Whitening seed = 0x%04X)\n", FSK_WHITENING_SEED );
    }
    HAL_DBG_TRACE_PRINTF( "\n" );
}*/

void print_driver_version( void )
{
    HAL_DBG_TRACE_INFO( "LR20XX driver version: %s\n", lr20xx_driver_version_get_version_string( ) );
}

void radio_on_dio_irq( void* context )
{
    irq_fired = true;
}
void on_tx_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_done( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_tx_timeout( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_preamble_detected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_syncword_header_valid( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_lora_header_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_len_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_crc_error( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_cad_done_undetected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_cad_done_detected( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_timestamp( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_rx_fifo_threshold( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}
void on_tx_fifo_threshold( void )
{
    HAL_DBG_TRACE_INFO( "No IRQ routine defined\n" );
}

/* --- EOF ------------------------------------------------------------------ */
