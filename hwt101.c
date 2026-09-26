#include "stm32f4xx.h"
#include "straight_control.h"
#include "hwt101.h"

extern volatile uint32_t clockMs;
volatile float imuYaw;
volatile uint32_t imuStamp;
volatile uint8_t imuValid;
static uint8_t imuFrame[11], imuLength;

void USART2_IRQHandler(void)
{
    uint8_t byte, i;
    float yaw;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(USART2);
    if (imuLength == 0U && byte != 0x55U) return;
    imuFrame[imuLength++] = byte;
    if (imuLength < 11U) return;
    if (decodeYaw(imuFrame, &yaw)) {
        imuYaw = yaw;
        imuStamp = clockMs;
        imuValid = 1U;
        imuLength = 0U;
    } else {
        /* Sliding window recovers after a dropped byte or another frame type. */
        for (i = 1U; i < 11U; ++i) imuFrame[i - 1U] = imuFrame[i];
        imuLength = 10U;
    }
}

void hwt101Init(uint32_t baud)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    USART_Cmd(USART2, DISABLE);
    imuValid = 0U;
    imuLength = 0U;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &gpio);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
    USART_StructInit(&uart);
    uart.USART_BaudRate = baud;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &uart);
    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    (void)USART2->SR;
    (void)USART2->DR;
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}
