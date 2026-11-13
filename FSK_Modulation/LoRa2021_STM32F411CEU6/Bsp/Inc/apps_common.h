/*!
 * @file      apps_common.h
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

#ifndef APPS_COMMON_H
#define APPS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "apps_configuration.h"
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_system_types.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define RX_CONTINUOUS 0xFFFFFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

typedef struct apps_common_lr20xx_irq_cfg_s
{
    lr20xx_system_irq_mask_t irq_mask;
    lr20xx_system_dio_t      dio;
} apps_common_lr20xx_irq_cfg_t;

typedef struct lr20xx_hal_context_s lr20xx_hal_context_t;


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Interface to initialise and return the lr20xx_hal_context_t object corresponding to the board
 *
 * @warning It is up to the caller to ensure the context pointer stays valid for the life duration of the
 * lr20xx_hal_context_t created by this call
 *
 * @returns Pointer to the lr20xx_hal_context_t object of the board
 */
lr20xx_hal_context_t* apps_common_lr20xx_get_context( );

/*!
 * @brief Initialize the system configuration of the transceiver
 */
void apps_common_shield_init( void );

/*!
 * @brief Initialize the system configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_lr20xx_system_init( const lr20xx_hal_context_t* context );

/*!
 * @brief Initialize the radio configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_lr20xx_radio_init( const void* context );

/*!
 * @brief Initialize the radio configuration of the transceiver for dbpsk only
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] context  Length of the payload
 */
void apps_common_lr20xx_radio_dbpsk_init( const void* context, const uint8_t payload_len );

/*!
 * @brief read the current length sirectly from the header buffer.
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] length  Pointer to the length variable where the length will be stored to
 */
void apps_common_lr20xx_read_length_from_header( void* context, uint16_t* length );

/**
 * @brief Fetch the version from the LR11xx and print it on the log interface
 *
 * @param [in] context  Pointer to the radio context
 *
 * @see apps_common_lr20xx_fetch_version, apps_common_lr20xx_print_version
 */
void apps_common_lr20xx_fetch_and_print_version( const lr20xx_hal_context_t* context );

/**
 * @brief Fetch the version from the LR11xx
 *
 * @param [in] context  Pointer to the radio context
 * @param [out] version  Version of the LR11XX
 *
 * @see apps_common_lr20xx_fetch_and_print_version, apps_common_lr20xx_print_version
 */
void apps_common_lr20xx_fetch_version( const lr20xx_hal_context_t* context, lr20xx_system_version_t* version );

/**
 * @brief Inits the dio pins as specified
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] irq_cfg  The configuration to be set to the irq pins
 * @param [in] irq_cfg_len  The length of the configuration to be set to the irq pins
 *
 * @see apps_common_lr20xx_fetch_and_print_version, apps_common_lr20xx_print_version
 */
void apps_common_lr20xx_set_dio_irq_params( const void* context, const apps_common_lr20xx_irq_cfg_t* irq_cfg,
                                            const uint8_t irq_cfg_len );

/**
 * @brief Print the version from the LR11xx on the log interface
 *
 * @param [in] version  Version of the LR11XX
 *
 * @see apps_common_lr20xx_fetch_and_print_version, apps_common_lr20xx_fetch_version
 */
void apps_common_lr20xx_print_version( const lr20xx_system_version_t* version );

/**
 * @brief Print the version of SDK and LR11XX driver
 *
 */
void apps_common_print_sdk_driver_version( void );

void apps_common_lr20xx_handle_pre_tx( void );
void apps_common_lr20xx_handle_post_tx( void );
void apps_common_lr20xx_handle_pre_rx( void );
void apps_common_lr20xx_handle_post_rx( void );

/*!
 * @brief Interface to read bytes from rx buffer
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] buffer Pointer to a byte array to be filled with content from memory. Its size must be enough to contain
 * at least length bytes
 * @param [in] buffer_length Length of the buffer to contain payload data
 * @param [in] size Number of bytes to read from memory
 */
void apps_common_lr20xx_receive( const void* context, uint8_t* buffer, uint16_t buffer_length, uint16_t* size );

/*!
 * @brief Interface to lr11xx interrupt processing routine
 *
 * This function fetched the IRQ mask from the lr11xx and process the raised IRQ if the corresponding bit is also set in
 * irq_filter.
 * The argument irq_filter allows to not process an IRQ even if it is raised by the lr11xx.
 *
 * @warning This function must be called from the main loop of project to dispense all the lr11xx interrupt routine
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] irq_filter_mask  Mask of IRQ to process
 * @param [in] log_enable  True if logs are anable on this function. Disable logs for best performances.
 */
void apps_common_lr20xx_irq_process( const void* context, lr20xx_system_irq_mask_t irq_filter_mask, bool log_enable );

/*!
 * @brief Evaluates if the last issued command has been correctly interpreted by the transceiver. It has to be run after
 * issueing a command.
 *
 * @param [in] context  Pointer to the radio context
 * @returns OK if last command has been correctly parsed, ERROR otherwise.
 */
lr20xx_status_t apps_common_lr20xx_radio_get_last_cmd_status( const void* context );

/*!
 * @brief Computes time on air, packet type agnostic
 */
uint32_t get_time_on_air_in_ms( void );

/*!
 * @brief A function to get the value for low data rate optimization setting
 *
 * @param [in] sf  LoRa Spreading Factor
 * @param [in] bw  LoRa Bandwidth
 */
inline static uint8_t apps_common_compute_lora_ppm( const lr20xx_radio_lora_sf_t sf, const lr20xx_radio_lora_bw_t bw )
{
    switch( bw )
    {
    case LR20XX_RADIO_LORA_BW_500:
        return LR20XX_RADIO_LORA_NO_PPM;

    case LR20XX_RADIO_LORA_BW_250:
        if( sf == LR20XX_RADIO_LORA_SF12 )
        {
            return LR20XX_RADIO_LORA_PPM_1_4;
        }
        else
        {
            return LR20XX_RADIO_LORA_NO_PPM;
        }

    case LR20XX_RADIO_LORA_BW_812:
    case LR20XX_RADIO_LORA_BW_406:
    case LR20XX_RADIO_LORA_BW_203:
    case LR20XX_RADIO_LORA_BW_125:
        if( ( sf == LR20XX_RADIO_LORA_SF12 ) || ( sf == LR20XX_RADIO_LORA_SF11 ) )
        {
            return LR20XX_RADIO_LORA_PPM_1_4;
        }
        else
        {
            return LR20XX_RADIO_LORA_NO_PPM;
        }

    case LR20XX_RADIO_LORA_BW_62:
        if( ( sf == LR20XX_RADIO_LORA_SF12 ) || ( sf == LR20XX_RADIO_LORA_SF11 ) || ( sf == LR20XX_RADIO_LORA_SF10 ) )
        {
            return LR20XX_RADIO_LORA_PPM_1_4;
        }
        else
        {
            return LR20XX_RADIO_LORA_NO_PPM;
        }

    case LR20XX_RADIO_LORA_BW_41:
        if( ( sf == LR20XX_RADIO_LORA_SF12 ) || ( sf == LR20XX_RADIO_LORA_SF11 ) || ( sf == LR20XX_RADIO_LORA_SF10 ) ||
            ( sf == LR20XX_RADIO_LORA_SF9 ) )
        {
            return LR20XX_RADIO_LORA_PPM_1_4;
        }
        else
        {
            return LR20XX_RADIO_LORA_NO_PPM;
        }

    case LR20XX_RADIO_LORA_BW_31:
    case LR20XX_RADIO_LORA_BW_20:
    case LR20XX_RADIO_LORA_BW_15:
    case LR20XX_RADIO_LORA_BW_10:
        // case LR11XX_RADIO_LORA_BW_7:
        return LR20XX_RADIO_LORA_PPM_1_4;

    default:
        return LR20XX_RADIO_LORA_NO_PPM;
    }
}

#ifdef __cplusplus
}
#endif

#endif  // APPS_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
