/*!
 * \file      lr20xx_hal.c
 *
 * \brief     Implements the lr20xx radio HAL functions on top of the bsp_ layer
 *
 * Слой не знает про HAL микроконтроллера: SPI, GPIO и время берутся из board.h (bsp_*).
 * Ничего не печатает; ошибки SPI и таймаут BUSY возвращаются как LR20XX_HAL_STATUS_ERROR.
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#include <stdint.h>
#include <stddef.h>

#include "lr20xx_hal.h"
#include "board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS ----------------------------------------------------------
 */

/* Максимальное время ожидания BUSY = low, мс. Нужно только чтобы не зависнуть при
 * обрыве линии или выключенном модуле. Значение с запасом (старт после reset,
 * переход в TX/RX из standby занимают миллисекунды), при необходимости уменьшить. */
#ifndef LR20XX_HAL_BUSY_TIMEOUT_MS
#define LR20XX_HAL_BUSY_TIMEOUT_MS 500u
#endif

/* Длительность низкого уровня NRESET, мс. */
#define LR20XX_HAL_RESET_PULSE_MS 2u

/* Длительность низкого уровня NSS для выхода из Sleep, мс (даташит: не менее 100 мкс). */
#define LR20XX_HAL_WAKEUP_PULSE_MS 2u

/* Сколько байт статуса (Stat15:8, Stat7:0) идёт перед данными в ответе на чтение (даташит, рис. 5-3). */
#define LR20XX_HAL_STATUS_BYTES 2u

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS -------------------------------------------------------
 */

/**
 * @brief Ждать, пока BUSY станет low
 * @return 0 = готов, -1 = таймаут
 */
static int prv_wait_on_busy( void )
{
    const uint32_t t0 = bsp_get_ms( );

    while( bsp_radio_busy_read( ) != 0u )
    {
        if( ( uint32_t ) ( bsp_get_ms( ) - t0 ) >= LR20XX_HAL_BUSY_TIMEOUT_MS )
        {
            return -1;
        }
    }
    return 0;
}

/**
 * @brief Закрыть кадр SPI (NSS = high) и преобразовать код bsp в статус HAL
 */
static lr20xx_hal_status_t prv_end_frame( int rc )
{
    bsp_radio_nss_set( 1u );
    return ( rc == 0 ) ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr20xx_hal_status_t lr20xx_hal_reset( const void* context )
{
    ( void ) context;

    bsp_radio_reset_set( 0u );
    bsp_delay_ms( LR20XX_HAL_RESET_PULSE_MS );
    bsp_radio_reset_set( 1u );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup( const void* context )
{
    ( void ) context;

    bsp_radio_nss_set( 0u );
    bsp_delay_ms( LR20XX_HAL_WAKEUP_PULSE_MS );
    bsp_radio_nss_set( 1u );

    return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    int rc;

    ( void ) context;

    if( ( command == NULL ) || ( command_length == 0u ) )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if( prv_wait_on_busy( ) != 0 )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }

    bsp_radio_nss_set( 0u );

    rc = bsp_spi_write( command, command_length );
    if( ( rc == 0 ) && ( data != NULL ) && ( data_length > 0u ) )
    {
        rc = bsp_spi_write( data, data_length );
    }

    return prv_end_frame( rc );
}

lr20xx_hal_status_t lr20xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    uint8_t status_bytes[LR20XX_HAL_STATUS_BYTES];
    int     rc;

    ( void ) context;

    if( ( command == NULL ) || ( command_length == 0u ) )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* Кадр 1: команда */
    if( prv_wait_on_busy( ) != 0 )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }
    bsp_radio_nss_set( 0u );
    rc = prv_end_frame( bsp_spi_write( command, command_length ) );
    if( rc != LR20XX_HAL_STATUS_OK )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* Кадр 2: статус (отбрасывается) и данные ответа; на MOSI уходят нули (NOP) */
    if( prv_wait_on_busy( ) != 0 )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }
    bsp_radio_nss_set( 0u );

    rc = bsp_spi_read( status_bytes, LR20XX_HAL_STATUS_BYTES );
    if( ( rc == 0 ) && ( data != NULL ) && ( data_length > 0u ) )
    {
        rc = bsp_spi_read( data, data_length );
    }

    return prv_end_frame( rc );
}

lr20xx_hal_status_t lr20xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    ( void ) context;

    if( ( data == NULL ) || ( data_length == 0u ) )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if( prv_wait_on_busy( ) != 0 )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }

    bsp_radio_nss_set( 0u );
    return prv_end_frame( bsp_spi_read( data, data_length ) );
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    int rc;

    ( void ) context;

    if( ( command == NULL ) || ( command_length == 0u ) || ( data == NULL ) || ( data_length == 0u ) )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }
    if( prv_wait_on_busy( ) != 0 )
    {
        return LR20XX_HAL_STATUS_ERROR;
    }

    /* Один кадр: опкод, затем данные FIFO сразу за ним (даташит, рис. 5-4).
     * Статус приходит на MISO одновременно с опкодом и здесь не нужен. */
    bsp_radio_nss_set( 0u );

    rc = bsp_spi_write( command, command_length );
    if( rc == 0 )
    {
        rc = bsp_spi_read( data, data_length );
    }

    return prv_end_frame( rc );
}

/* --- EOF ------------------------------------------------------------------ */
