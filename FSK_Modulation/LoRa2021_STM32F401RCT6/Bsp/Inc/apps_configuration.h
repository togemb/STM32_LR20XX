/*!
 * @file      apps_configuration.h
 *
 * @brief     Common configuration
 *
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

#ifndef APPS_CONFIGURATION_H
#define APPS_CONFIGURATION_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "lr20xx_radio_lora_types.h"
#include "lr20xx_radio_fsk_types.h"
#include "lr20xx_radio_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*!
 * @brief General parameters
 */
#ifndef PACKET_TYPE
#define PACKET_TYPE LR20XX_RADIO_COMMON_PKT_TYPE_FSK
#endif
#ifndef RF_FREQ_IN_HZ
#define RF_FREQ_IN_HZ 2440000000u 
#endif
#ifndef TX_OUTPUT_POWER_HALF_DBM_STEPS
#define TX_OUTPUT_POWER_HALF_DBM_STEPS \
    (-18) /* in 0.5dBm steps, range [-11, +36] for sub-G, range [-18, 4] for 2.4G ( HF_PA ) */
#endif

#ifndef PA_RAMP_TIME
#define PA_RAMP_TIME LR20XX_RADIO_COMMON_RAMP_32_US
#endif
#ifndef FALLBACK_MODE
#define FALLBACK_MODE LR20XX_RADIO_FALLBACK_STDBY_RC
#endif
#ifndef ENABLE_RX_BOOST_MODE
#define ENABLE_RX_BOOST_MODE false
#endif
#ifndef PAYLOAD_LENGTH
#define PAYLOAD_LENGTH 11
#endif

/*!
 * @brief Modulation parameters for LoRa packets
 */
#ifndef LORA_SPREADING_FACTOR
#define LORA_SPREADING_FACTOR LR20XX_RADIO_LORA_SF7
#endif
#ifndef LORA_BANDWIDTH
#define LORA_BANDWIDTH LR20XX_RADIO_LORA_BW_406
#endif
#ifndef LORA_CODING_RATE
#define LORA_CODING_RATE LR20XX_RADIO_LORA_CR_4_5
#endif

/*!
 * @brief Packet parameters for LoRa packets
 */
#ifndef LORA_PREAMBLE_LENGTH
#define LORA_PREAMBLE_LENGTH 12
#endif
#ifndef LORA_PKT_LEN_MODE
#define LORA_PKT_LEN_MODE LR20XX_RADIO_LORA_PKT_EXPLICIT
#endif
#ifndef LORA_IQ
#define LORA_IQ LR20XX_RADIO_LORA_IQ_STANDARD
#endif
#ifndef LORA_CRC
#define LORA_CRC LR20XX_RADIO_LORA_CRC_ENABLED
#endif

/*!
 * @brief LoRa sync word
 */
#ifndef LORA_SYNCWORD
#define LORA_SYNCWORD 0x12  // 0x12 Private Network, 0x34 Public Network
#endif


/*!
 * @brief Parameters for FSK packets (variable-length packet format)
 *
 * Для смены скорости достаточно изменить FSK_BITRATE_BPS: девиация и полоса приёма
 * пересчитываются автоматически (h = 0.5, полоса = 2*fdev + bitrate + запас).
 * Допустимые значения и ограничения проверяются при компиляции в lr2021.c.
 *
 *   bitrate 500 kbps  -> fdev 125 kHz, BW >= 800 kHz  (BW_888)
 *   bitrate 1 Mbps    -> fdev 250 kHz, BW >= 1.55 MHz (BW_2222)
 *   bitrate 2 Mbps    -> fdev 500 kHz, BW >= 3.05 MHz (BW_3076) - максимум чипа
 */

/* Битрейт, бит/с. Диапазон чипа: 500 .. 2 000 000 */
#ifndef FSK_BITRATE_BPS
#define FSK_BITRATE_BPS 500000u
#endif

/* Девиация, Гц. По умолчанию индекс модуляции h = 2*fdev/bitrate = 0.5 (как BLE LE 2M).
 * Диапазон чипа: 600 .. 500 000 Гц. */
#ifndef FSK_FDEV_HZ
#define FSK_FDEV_HZ ( FSK_BITRATE_BPS / 4u )
#endif

/* Гауссов фильтр. BT = 0.5 сужает спектр (меньше помех соседям и от соседей). */
#ifndef FSK_PULSE_SHAPE
#define FSK_PULSE_SHAPE LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_0_5
#endif

/* Запас полосы приёма на уход частоты (кварцы TX и RX), Гц. */
#ifndef FSK_FREQ_ERR_MARGIN_HZ
#define FSK_FREQ_ERR_MARGIN_HZ 50000u
#endif

/* Минимальная полоса приёма, Гц. Выбирается ближайшая табличная полоса не меньше этого значения
 * (даташит LR20xx, 11.1.3: BW_min >= 2*Fdev + bitrate + Freq_err_max). */
#ifndef FSK_RX_BW_MIN_HZ
#define FSK_RX_BW_MIN_HZ ( 2u * FSK_FDEV_HZ + FSK_BITRATE_BPS + FSK_FREQ_ERR_MARGIN_HZ )
#endif

/* Преамбула: длина при передаче и длина детектора при приёме, бит.
 * Для osr = 4 (h < 0.85) даташит рекомендует не менее 20 бит детектирования. */
#ifndef FSK_PREAMBLE_TX_BITS
#define FSK_PREAMBLE_TX_BITS 48u
#endif
#ifndef FSK_PREAMBLE_DETECTOR
#define FSK_PREAMBLE_DETECTOR LR20XX_RADIO_FSK_PREAMBLE_DETECTOR_24_BITS
#endif

/* Синхрослово: 32 бита, сбалансировано, низкие боковые лепестки автокорреляции. */
#ifndef FSK_SYNCWORD
#define FSK_SYNCWORD 0x930B51DEu
#endif
#ifndef FSK_SYNCWORD_BITS
#define FSK_SYNCWORD_BITS 32u
#endif

/* Максимальная длина полезной нагрузки, байт. Заголовок длины 8 бит -> не более 255.
 * Для приёма это порог: пакет длиннее вызывает LEN_ERROR. */
#ifndef FSK_MAX_PAYLOAD_BYTES
#define FSK_MAX_PAYLOAD_BYTES 255u
#endif

/* CRC-32 (IEEE 802.3), 4 байта. В режиме variable-length CRC считается по заголовку и нагрузке. */
#ifndef FSK_CRC_TYPE
#define FSK_CRC_TYPE LR20XX_RADIO_FSK_CRC_4_BYTES
#endif
#ifndef FSK_CRC_POLYNOMIAL
#define FSK_CRC_POLYNOMIAL 0x04C11DB7u
#endif
#ifndef FSK_CRC_SEED
#define FSK_CRC_SEED 0xFFFFFFFFu
#endif

/* Whitening (DC-free): выравнивает спектр при длинных сериях одинаковых бит. */
#ifndef FSK_WHITENING
#define FSK_WHITENING LR20XX_RADIO_FSK_WHITENING_ON
#endif
#ifndef FSK_WHITENING_SEED
#define FSK_WHITENING_SEED 0x01FFu
#endif

/* Таймаут передачи одного пакета, мс. Должен быть больше времени пакета в эфире. */
#ifndef FSK_TX_TIMEOUT_MS
#define FSK_TX_TIMEOUT_MS 100u
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // APPS_CONFIGURATION_H

/* --- EOF ------------------------------------------------------------------ */
