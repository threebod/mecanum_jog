#include "stm32f4xx.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"

#include <string.h>

#define COMMAND_BAUD_RATE  115200U
#define RX_LINE_SIZE       24U

#define MOTOR_MIN_ID       1U
#define MOTOR_MAX_ID       4U
#define TEST_MOTOR_ID      5U
#define MOTOR_TEST_SPEED   30U
#define MOTOR_TEST_ACCEL   50U
#define MOTOR_TEST_PULSES  160U

typedef enum {
    DIRECTION_FORWARD = 0,
    DIRECTION_BACK,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} MecanumDirection;

/* Index 0 is unused; indexes 1..4 are the motor CAN addresses. */
static const uint8_t motorDirections[4][5] = {
    {1U, 1U, 0U, 0U, 1U},
    {0U, 0U, 1U, 1U, 0U},
    {1U, 1U, 1U, 0U, 0U},
    {0U, 0U, 0U, 1U, 1U}
};

static volatile char rxLine[RX_LINE_SIZE];
static volatile uint8_t rxLength;
static volatile uint8_t rxReady;
static volatile uint8_t emergencyStop;
static uint8_t armed;

static void serialSendChar(char value)
{
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(UART5, (uint16_t)(uint8_t)value);
}

static void serialSendString(const char *text)
{
    while (*text != '\0') {
        serialSendChar(*text++);
    }
    while (USART_GetFlagStatus(UART5, USART_FLAG_TC) == RESET) {
    }
}

static void serialInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC |
                           RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = COMMAND_BAUD_RATE;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &usart);

    nvic.NVIC_IRQChannel = UART5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 2U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);
}

static void stopAllMotors(void)
{
    uint8_t id;

    for (id = MOTOR_MIN_ID; id <= TEST_MOTOR_ID; ++id) {
        Emm_V5_Stop_Now(id, false);
        delay_ms(2U);
    }
    armed = 0U;
}

static void setAllMotorsEnabled(bool enabled)
{
    uint8_t id;

    for (id = MOTOR_MIN_ID; id <= MOTOR_MAX_ID; ++id) {
        Emm_V5_En_Control(id, enabled, false);
        delay_ms(2U);
    }
}

static void printHelp(void)
{
    serialSendString("\r\nMecanum jog commands:\r\n");
    serialSendString("  arm       authorize ONE enable or motion command\r\n");
    serialSendString("  enable    enable motors 1..4 (armed)\r\n");
    serialSendString("  disable   disable motors 1..4 and disarm\r\n");
    serialSendString("  enable5   enable diagnostic motor 5 (armed)\r\n");
    serialSendString("  motor5 0  jog motor 5 in direction 0 (armed)\r\n");
    serialSendString("  motor5 1  jog motor 5 in direction 1 (armed)\r\n");
    serialSendString("  disable5  stop and disable motor 5\r\n");
    serialSendString("  W/S/A/D   forward/back/left/right\r\n");
    serialSendString("  X, stop   stop motors and disarm\r\n");
    serialSendString("  !         emergency stop (no Enter needed)\r\n");
    serialSendString("  help      show this help\r\n");
}

static uint8_t motionInterrupted(void)
{
    return emergencyStop != 0U;
}

static void startJog(MecanumDirection direction, const char *name)
{
    uint8_t id;

    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return;
    }
    armed = 0U;

    for (id = MOTOR_MIN_ID; id <= MOTOR_MAX_ID; ++id) {
        if (motionInterrupted()) {
            return;
        }
        Emm_V5_Pos_Control(id, motorDirections[(uint8_t)direction][id],
                           MOTOR_TEST_SPEED, MOTOR_TEST_ACCEL,
                           MOTOR_TEST_PULSES, false, true);
        delay_ms(2U);
    }

    if (motionInterrupted()) {
        return;
    }
    Emm_V5_Synchronous_motion(0x00);
    serialSendString("OK: chassis ");
    serialSendString(name);
    serialSendString(", auto-disarmed\r\n");
}

static void startMotor5Jog(uint8_t direction)
{
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return;
    }
    armed = 0U;

    Emm_V5_Pos_Control(TEST_MOTOR_ID, direction,
                       MOTOR_TEST_SPEED, MOTOR_TEST_ACCEL,
                       MOTOR_TEST_PULSES, false, false);
    serialSendString("OK: motor 5 direction ");
    serialSendString(direction == 0U ? "0" : "1");
    serialSendString(", auto-disarmed\r\n");
}

static void processCommand(const char *command)
{
    if (strcmp(command, "arm") == 0) {
        armed = 1U;
        serialSendString("ARMED for one enable or motion command\r\n");
    } else if (strcmp(command, "enable") == 0) {
        if (!armed) {
            serialSendString("ERR: send 'arm' first\r\n");
            return;
        }
        armed = 0U;
        setAllMotorsEnabled(true);
        serialSendString("OK: motors 1..4 enabled, auto-disarmed\r\n");
    } else if (strcmp(command, "disable") == 0) {
        setAllMotorsEnabled(false);
        armed = 0U;
        serialSendString("OK: motors 1..4 disabled and disarmed\r\n");
    } else if (strcmp(command, "enable5") == 0) {
        if (!armed) {
            serialSendString("ERR: send 'arm' first\r\n");
            return;
        }
        armed = 0U;
        Emm_V5_En_Control(TEST_MOTOR_ID, true, false);
        serialSendString("OK: motor 5 enabled, auto-disarmed\r\n");
    } else if (strcmp(command, "motor5 0") == 0) {
        startMotor5Jog(0U);
    } else if (strcmp(command, "motor5 1") == 0) {
        startMotor5Jog(1U);
    } else if (strcmp(command, "disable5") == 0) {
        Emm_V5_Stop_Now(TEST_MOTOR_ID, false);
        delay_ms(2U);
        Emm_V5_En_Control(TEST_MOTOR_ID, false, false);
        armed = 0U;
        serialSendString("OK: motor 5 stopped, disabled and disarmed\r\n");
    } else if (strcmp(command, "W") == 0 || strcmp(command, "w") == 0) {
        startJog(DIRECTION_FORWARD, "forward");
    } else if (strcmp(command, "S") == 0 || strcmp(command, "s") == 0) {
        startJog(DIRECTION_BACK, "back");
    } else if (strcmp(command, "A") == 0 || strcmp(command, "a") == 0) {
        startJog(DIRECTION_LEFT, "left");
    } else if (strcmp(command, "D") == 0 || strcmp(command, "d") == 0) {
        startJog(DIRECTION_RIGHT, "right");
    } else if (strcmp(command, "X") == 0 || strcmp(command, "x") == 0 ||
               strcmp(command, "stop") == 0) {
        stopAllMotors();
        serialSendString("STOPPED and disarmed\r\n");
    } else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        printHelp();
    } else if (command[0] != '\0') {
        serialSendString("ERR: unknown command; send 'help'\r\n");
    }
}

void UART5_IRQHandler(void)
{
    char received;

    if (USART_GetITStatus(UART5, USART_IT_RXNE) == RESET) {
        return;
    }

    received = (char)USART_ReceiveData(UART5);
    if (received == '!') {
        emergencyStop = 1U;
        rxLength = 0U;
        rxReady = 0U;
    } else if (!emergencyStop &&
               (received == '\r' || received == '\n') &&
               rxLength > 0U && !rxReady) {
        rxLine[rxLength] = '\0';
        rxReady = 1U;
    } else if (!emergencyStop && received != '\r' && received != '\n' &&
               !rxReady) {
        if (rxLength < RX_LINE_SIZE - 1U) {
            rxLine[rxLength++] = received;
        } else {
            rxLength = 0U;
        }
    }

    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
}

int main(void)
{
    char command[RX_LINE_SIZE];

    delay_init(168U);
    board_init();
    serialInit();

    stopAllMotors();
    serialSendString("\r\nYYB mecanum jog ready; motors 1..5 stopped; disarmed.\r\n");
    printHelp();

    for (;;) {
        if (emergencyStop) {
            stopAllMotors();
            __disable_irq();
            emergencyStop = 0U;
            rxLength = 0U;
            rxReady = 0U;
            __enable_irq();
            serialSendString("EMERGENCY STOP; motors 1..5 stopped; disarmed\r\n");
        }

        if (rxReady) {
            __disable_irq();
            strcpy(command, (const char *)rxLine);
            rxLength = 0U;
            rxReady = 0U;
            __enable_irq();
            processCommand(command);
        }
    }
}
