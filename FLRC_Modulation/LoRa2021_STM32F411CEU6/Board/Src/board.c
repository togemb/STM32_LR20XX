#include "board.h"

#include <stdio.h>

#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#define BOARD_SPI_TIMEOUT_MS   20u
#define BOARD_DELAY_CHUNK_US   500u   /* меньше периода TIM2 (1000 мкс) */

/* ---- Инициализация ---------------------------------------------------- */

void board_init(void)
{
    /* TIM2: 1 тик = 1 мкс. Прерывание не включаем, счётчик только читаем. */
    HAL_TIM_Base_Start(&htim2);
}

/* ---- Время ------------------------------------------------------------ */

static uint32_t tim_elapsed_us(uint32_t start)
{
    uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    /* Работает и для периода 1000 мкс, и для свободного 32-битного счёта */
    return (now >= start) ? (now - start) : (arr - start + 1u + now);
}

static void delay_chunk_us(uint32_t us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);

    while (tim_elapsed_us(start) < us) {
    }
}

void board_delay_us(uint32_t us)
{
    while (us > BOARD_DELAY_CHUNK_US) {
        delay_chunk_us(BOARD_DELAY_CHUNK_US);
        us -= BOARD_DELAY_CHUNK_US;
    }
    delay_chunk_us(us);
}

void board_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

uint32_t board_get_ms(void)
{
    return HAL_GetTick();
}

/* ---- Линии радиомодуля ------------------------------------------------ */

void board_radio_ce(bool level)
{
    HAL_GPIO_WritePin(LORA_CE_GPIO_Port, LORA_CE_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_radio_nrst(bool level)
{
    HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void board_radio_nss(bool level)
{
    HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool board_radio_busy(void)
{
    return HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin) == GPIO_PIN_SET;
}

bool board_radio_irq(void)
{
    return HAL_GPIO_ReadPin(LORA_IRQ_GPIO_Port, LORA_IRQ_Pin) == GPIO_PIN_SET;
}

/* Прерывание по фронту IRQ только ставит флаг. SPI-обмен с радио в прерывании не делаем. */
static volatile bool radio_irq_flag;

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
    if (gpio_pin == LORA_IRQ_Pin) {
        radio_irq_flag = true;
    }
}

bool board_radio_irq_take(void)
{
    if (radio_irq_flag) {
        radio_irq_flag = false;
        return true;
    }
    return false;
}

/* ---- SPI -------------------------------------------------------------- */

bool board_radio_spi_write(const uint8_t *data, uint16_t len)
{
    return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, BOARD_SPI_TIMEOUT_MS) == HAL_OK;
}

bool board_radio_spi_read(uint8_t *data, uint16_t len)
{
    static const uint8_t zeros[32] = { 0 };

    while (len > 0u) {
        uint16_t n = (len > sizeof(zeros)) ? (uint16_t)sizeof(zeros) : len;

        if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)zeros, data, n, BOARD_SPI_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
        data += n;
        len  -= n;
    }
    return true;
}

/* ---- printf -> USART2 ------------------------------------------------- */

int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;

    HAL_UART_Transmit(&huart2, &c, 1, 10);
    return ch;
}
