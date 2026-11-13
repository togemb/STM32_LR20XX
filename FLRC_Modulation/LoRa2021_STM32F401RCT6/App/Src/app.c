#include "app.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "radio.h"

/* Роль платы: 0 - передатчик (раз в секунду шлёт строку), 1 - приёмник. */
#define APP_ROLE_RX         0

/* Строка для отправки. К ней добавляется номер передачи: "Hello FLRC #1", "Hello FLRC #2"... */
#define APP_MESSAGE         "Hello FLRC"
#define APP_MESSAGE_MAX     64u     /* размер буфера строки передачи, включая завершающий ноль */

#define APP_TX_PERIOD_MS    1000u   /* период отправки, от старта одной передачи до старта следующей */
#define APP_ERROR_RETRY_MS  2000u   /* через сколько повторить запуск радио после ошибки */

typedef enum {
    APP_STATE_INIT,      /* запуск радио */
    APP_STATE_SEND,      /* собрать строку и запустить передачу */
    APP_STATE_WAIT_TX,   /* ждать окончания передачи */
    APP_STATE_PAUSE,     /* ждать начала следующего периода */
    APP_STATE_RX_START,  /* запуск непрерывного приёма */
    APP_STATE_RX_WAIT,   /* ждать пакеты */
    APP_STATE_ERROR      /* пауза перед повторным запуском */
} app_state_t;

/* Печать значения в половинках единицы: -171 -> "-85.5" */
static void print_half(int32_t value)
{
    uint32_t a = (uint32_t)((value < 0) ? -value : value);

    printf("%s%lu.%lu", (value < 0) ? "-" : "", (unsigned long)(a / 2u), (unsigned long)((a % 2u) * 5u));
}

/* Печать принятой строки. Строка приходит без завершающего нуля, а байты могут быть любыми:
 * непечатаемые заменяем точкой, чтобы мусор не ломал терминал. */
static void print_text(const uint8_t *p, uint8_t len)
{
    uint8_t i;

    for (i = 0; i < len; i++) {
        putchar(((p[i] >= 0x20u) && (p[i] <= 0x7Eu)) ? (int)p[i] : (int)'.');
    }
}

static void print_packet(const uint8_t *p, const radio_rx_info_t *info)
{
    printf("RX \"");
    print_text(p, info->len);
    printf("\" len %u, rssi ", (unsigned)info->len);
    if (info->rssi_overload) {
        printf("overload (too strong, move boards apart)");
    } else {
        print_half(info->rssi_half_dbm);
        printf(" dBm");
    }
    printf("\r\n");
}

void app(void)
{
    app_state_t     state = APP_STATE_INIT;
    radio_status_t  st;
    char            msg[APP_MESSAGE_MAX];
    int             msg_len;
    uint8_t         rx_buf[255];
    radio_rx_info_t rx_info;
    uint8_t         major = 0;
    uint8_t         minor = 0;
    uint32_t        tx_number = 0;   /* номер передачи, начиная с 1 */
    uint32_t        t_send = 0;      /* момент запуска передачи */
    uint32_t        t_error = 0;     /* момент входа в ошибку */

    board_init();
    printf("\r\n=== LoRa2021 start ===\r\n");

    while (1) {
        switch (state) {
        case APP_STATE_INIT:
            st = radio_lora_init();
            if (st == RADIO_OK) {
                st = radio_lora_get_version(&major, &minor);
            }
            if (st == RADIO_OK) {
                printf("radio ready, version %02X.%02X\r\n", major, minor);
#if APP_ROLE_RX
                st = radio_lora_rx_config();
#endif
            }
            if (st == RADIO_OK) {
#if APP_ROLE_RX
                state = APP_STATE_RX_START;
#else
                state = APP_STATE_SEND;
#endif
            } else {
                printf("radio init failed, code %d\r\n", (int)st);
                t_error = board_get_ms();
                state   = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_SEND:
            tx_number++;
            msg_len = snprintf(msg, sizeof(msg), APP_MESSAGE " #%lu", (unsigned long)tx_number);
            if ((msg_len <= 0) || ((unsigned)msg_len >= sizeof(msg))) {
                msg_len = (int)strlen(msg);   /* строка не поместилась: шлём обрезанную */
            }
            t_send = board_get_ms();
            st     = radio_lora_send((const uint8_t *)msg, (uint8_t)msg_len);
            if (st == RADIO_OK) {
                state = APP_STATE_WAIT_TX;
            } else {
                printf("TX #%lu start failed, code %d\r\n", (unsigned long)tx_number, (int)st);
                t_error = board_get_ms();
                state   = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_WAIT_TX:
            switch (radio_lora_tx_poll()) {
            case RADIO_TX_BUSY:
                break;
            case RADIO_TX_DONE:
                printf("TX \"%s\" done, %lu ms\r\n", msg, (unsigned long)(board_get_ms() - t_send));
                state = APP_STATE_PAUSE;
                break;
            case RADIO_TX_TIMEOUT:
                printf("TX #%lu timeout\r\n", (unsigned long)tx_number);
                state = APP_STATE_PAUSE;
                break;
            default:
                printf("TX #%lu failed\r\n", (unsigned long)tx_number);
                t_error = board_get_ms();
                state   = APP_STATE_ERROR;
                break;
            }
            break;

        case APP_STATE_PAUSE:
            if ((board_get_ms() - t_send) >= APP_TX_PERIOD_MS) {
                state = APP_STATE_SEND;
            }
            break;

        case APP_STATE_RX_START:
            st = radio_lora_rx_start();
            if (st == RADIO_OK) {
                printf("RX started\r\n");
                state = APP_STATE_RX_WAIT;
            } else {
                printf("RX start failed, code %d\r\n", (int)st);
                t_error = board_get_ms();
                state   = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_RX_WAIT:
            switch (radio_lora_rx_poll(rx_buf, sizeof(rx_buf), &rx_info)) {
            case RADIO_RX_NONE:
                break;
            case RADIO_RX_DONE:
                print_packet(rx_buf, &rx_info);
                break;
            case RADIO_RX_CRC_ERROR:
                printf("RX crc error\r\n");
                break;
            case RADIO_RX_LEN_ERROR:
                printf("RX length error\r\n");
                break;
            default:
                printf("RX failed, restarting radio\r\n");
                t_error = board_get_ms();
                state   = APP_STATE_ERROR;
                break;
            }
            break;

        case APP_STATE_ERROR:
            if ((board_get_ms() - t_error) >= APP_ERROR_RETRY_MS) {
                state = APP_STATE_INIT;
            }
            break;

        default:
            t_error = board_get_ms();
            state   = APP_STATE_ERROR;
            break;
        }
    }
}
