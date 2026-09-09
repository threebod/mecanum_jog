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
#define CAN_CHECK_TIMEOUT_MS  300U

#define SERVO_MIN_PULSE_US    500U
#define SERVO_PULSE_RANGE_US  2000U
#define SERVO_PERIOD_US       20000U
#define SERVO_SPEED_DPS       30U
#define SERVO_UPDATE_HZ       50U
#define SERVO_STEP_MDEG       ((SERVO_SPEED_DPS * 1000U) / SERVO_UPDATE_HZ)

typedef enum {
    DIRECTION_FORWARD = 0,
    DIRECTION_BACK,
    DIRECTION_LEFT,
    DIRECTION_RIGHT
} MecanumDirection;

typedef enum {
    COMMAND_OK = 0,
    COMMAND_FORMAT_ERROR,
    COMMAND_SERVO_ERROR,
    COMMAND_ANGLE_ERROR
} CommandResult;

typedef struct {
    uint8_t channel;
    uint16_t angle;
} ServoCommand;

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
static uint8_t servoTimerReady;
static volatile uint8_t servoChannelEnabled[3];
static volatile uint8_t servoChannelMoving[3];
static volatile uint8_t completedServoChannels;
static volatile uint32_t currentAngleMdeg[3];
static volatile uint32_t targetAngleMdeg[3];

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

static void serialSendHex4(uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    serialSendChar(digits[value & 0x0FU]);
}

static void serialSendHex8(uint8_t value)
{
    serialSendHex4(value >> 4);
    serialSendHex4(value);
}

static void serialSendHex32(uint32_t value)
{
    int8_t shift;

    for (shift = 28; shift >= 0; shift -= 4) {
        serialSendHex4((uint8_t)(value >> shift));
    }
}

static void serialSendUint(uint16_t value)
{
    char digits[5];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (count > 0U) {
        serialSendChar(digits[--count]);
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
    serialSendString("  cancheck5 query motor 5 CAN status\r\n");
    serialSendString("  servo N A set servo 2..4 to angle A\r\n");
    serialSendString("  W/S/A/D   forward/back/left/right\r\n");
    serialSendString("  X, stop   stop motors/servo motion and disarm\r\n");
    serialSendString("  !         emergency stop motors/servo motion\r\n");
    serialSendString("  help      show this help\r\n");
}

static uint8_t parseUint(const char **cursor, uint16_t *value)
{
    uint32_t result = 0U;
    uint8_t digitCount = 0U;

    while (**cursor == ' ') {
        (*cursor)++;
    }
    while (**cursor >= '0' && **cursor <= '9') {
        result = result * 10U + (uint32_t)(**cursor - '0');
        if (result > 65535U) {
            return 0U;
        }
        (*cursor)++;
        digitCount++;
    }
    if (digitCount == 0U) {
        return 0U;
    }
    *value = (uint16_t)result;
    return 1U;
}

static CommandResult parseServoCommand(const char *line, ServoCommand *command)
{
    const char *cursor = line;
    uint16_t channel;
    uint16_t angle;

    if (strncmp(cursor, "servo ", 6U) != 0) {
        return COMMAND_FORMAT_ERROR;
    }
    cursor += 6;
    if (!parseUint(&cursor, &channel) || !parseUint(&cursor, &angle)) {
        return COMMAND_FORMAT_ERROR;
    }
    while (*cursor == ' ') {
        cursor++;
    }
    if (*cursor != '\0') {
        return COMMAND_FORMAT_ERROR;
    }
    if (channel < 2U || channel > 4U) {
        return COMMAND_SERVO_ERROR;
    }
    if ((channel < 4U && angle > 270U) ||
        (channel == 4U && angle > 360U)) {
        return COMMAND_ANGLE_ERROR;
    }

    command->channel = (uint8_t)channel;
    command->angle = angle;
    return COMMAND_OK;
}

static uint16_t servoAngleToPulse(uint8_t channel, uint32_t angleMdeg)
{
    uint32_t maximumAngleMdeg = channel == 4U ? 360000U : 270000U;

    return (uint16_t)(SERVO_MIN_PULSE_US +
                      angleMdeg * SERVO_PULSE_RANGE_US / maximumAngleMdeg);
}

static void servoTimerInit(void)
{
    TIM_TimeBaseInitTypeDef timeBase;
    NVIC_InitTypeDef nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructInit(&timeBase);
    timeBase.TIM_Prescaler = 84U - 1U;
    timeBase.TIM_Period = SERVO_PERIOD_US - 1U;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &timeBase);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    servoTimerReady = 1U;
}

static void servoGpioEnable(uint8_t channel)
{
    GPIO_InitTypeDef gpio;
    uint16_t pin;
    uint8_t pinSource;

    if (channel == 2U) {
        pin = GPIO_Pin_1;
        pinSource = GPIO_PinSource1;
    } else if (channel == 3U) {
        pin = GPIO_Pin_2;
        pinSource = GPIO_PinSource2;
    } else {
        pin = GPIO_Pin_3;
        pinSource = GPIO_PinSource3;
    }

    GPIO_PinAFConfig(GPIOA, pinSource, GPIO_AF_TIM2);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
}

static void servoSetPulse(uint8_t channel, uint16_t pulse)
{
    TIM_OCInitTypeDef outputCompare;
    uint8_t index = (uint8_t)(channel - 2U);

    if (!servoTimerReady) {
        servoTimerInit();
    }
    if (!servoChannelEnabled[index]) {
        TIM_OCStructInit(&outputCompare);
        outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
        outputCompare.TIM_OutputState = TIM_OutputState_Enable;
        outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
        outputCompare.TIM_Pulse = pulse;

        if (channel == 2U) {
            TIM_OC2Init(TIM2, &outputCompare);
            TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
        } else if (channel == 3U) {
            TIM_OC3Init(TIM2, &outputCompare);
            TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
        } else {
            TIM_OC4Init(TIM2, &outputCompare);
            TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
        }
        TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
        servoGpioEnable(channel);
        servoChannelEnabled[index] = 1U;
    } else if (channel == 2U) {
        TIM_SetCompare2(TIM2, pulse);
    } else if (channel == 3U) {
        TIM_SetCompare3(TIM2, pulse);
    } else {
        TIM_SetCompare4(TIM2, pulse);
    }
}

static uint8_t servoSetTarget(uint8_t channel, uint16_t angle)
{
    uint8_t index = (uint8_t)(channel - 2U);
    uint32_t target = (uint32_t)angle * 1000U;

    if (!servoChannelEnabled[index]) {
        servoSetPulse(channel, servoAngleToPulse(channel, target));
        __disable_irq();
        currentAngleMdeg[index] = target;
        targetAngleMdeg[index] = target;
        servoChannelMoving[index] = 0U;
        __enable_irq();
        return 0U;
    }

    __disable_irq();
    targetAngleMdeg[index] = target;
    servoChannelMoving[index] = currentAngleMdeg[index] != target;
    completedServoChannels &= (uint8_t)~(1U << index);
    __enable_irq();
    return servoChannelMoving[index];
}

static void stopServoMotion(void)
{
    uint8_t index;

    __disable_irq();
    for (index = 0U; index < 3U; ++index) {
        targetAngleMdeg[index] = currentAngleMdeg[index];
        servoChannelMoving[index] = 0U;
    }
    completedServoChannels = 0U;
    __enable_irq();
}

static void processServoCommand(const char *commandText)
{
    ServoCommand command;
    CommandResult result = parseServoCommand(commandText, &command);
    uint16_t pulse;
    uint8_t moving;

    if (result == COMMAND_FORMAT_ERROR) {
        serialSendString("ERR format: servo <2|3|4> <angle>\r\n");
        return;
    }
    if (result == COMMAND_SERVO_ERROR) {
        serialSendString("ERR servo: 2..4\r\n");
        return;
    }
    if (result == COMMAND_ANGLE_ERROR) {
        serialSendString("ERR angle: servo 2/3=0..270, servo 4=0..360\r\n");
        return;
    }

    pulse = servoAngleToPulse(command.channel,
                              (uint32_t)command.angle * 1000U);
    moving = servoSetTarget(command.channel, command.angle);
    serialSendString(moving ? "MOVING servo=" : "OK servo=");
    serialSendUint(command.channel);
    serialSendString(moving ? " target=" : " angle=");
    serialSendUint(command.angle);
    if (moving) {
        serialSendString(" speed=30dps\r\n");
    } else {
        serialSendString(" pulse=");
        serialSendUint(pulse);
        serialSendString("us\r\n");
    }
}

static void reportCompletedServoMoves(void)
{
    uint8_t completed;
    uint8_t index;

    __disable_irq();
    completed = completedServoChannels;
    completedServoChannels = 0U;
    __enable_irq();

    for (index = 0U; index < 3U; ++index) {
        if ((completed & (uint8_t)(1U << index)) != 0U) {
            serialSendString("DONE servo=");
            serialSendUint((uint16_t)(index + 2U));
            serialSendString(" angle=");
            serialSendUint((uint16_t)(currentAngleMdeg[index] / 1000U));
            serialSendString("\r\n");
        }
    }
}

void TIM2_IRQHandler(void)
{
    uint8_t index;

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == RESET) {
        return;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    for (index = 0U; index < 3U; ++index) {
        uint32_t current;
        uint32_t target;

        if (!servoChannelEnabled[index] || !servoChannelMoving[index]) {
            continue;
        }
        current = currentAngleMdeg[index];
        target = targetAngleMdeg[index];
        if (current < target) {
            current = target - current <= SERVO_STEP_MDEG ?
                      target : current + SERVO_STEP_MDEG;
        } else {
            current = current - target <= SERVO_STEP_MDEG ?
                      target : current - SERVO_STEP_MDEG;
        }
        currentAngleMdeg[index] = current;
        servoSetPulse((uint8_t)(index + 2U),
                      servoAngleToPulse((uint8_t)(index + 2U), current));
        if (current == target) {
            servoChannelMoving[index] = 0U;
            completedServoChannels |= (uint8_t)(1U << index);
        }
    }
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
    serialSendString("TX queued: chassis ");
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
    serialSendString("TX queued: motor 5 direction ");
    serialSendString(direction == 0U ? "0" : "1");
    serialSendString(", auto-disarmed\r\n");
}

static void checkMotor5Can(void)
{
    uint32_t elapsed;
    uint32_t extId = 0U;
    uint8_t dlc = 0U;
    uint8_t data[8] = {0U};
    uint8_t i;
    uint8_t received = 0U;

    __disable_irq();
    can.rxFrameFlag = false;
    __enable_irq();
    Emm_V5_Read_Sys_Params(TEST_MOTOR_ID, S_FLAG);

    for (elapsed = 0U; elapsed < CAN_CHECK_TIMEOUT_MS; ++elapsed) {
        if (emergencyStop) {
            return;
        }
        if (can.rxFrameFlag) {
            __disable_irq();
            extId = can.CAN_RxMsg.ExtId;
            dlc = can.CAN_RxMsg.DLC;
            if (dlc > 8U) {
                dlc = 8U;
            }
            for (i = 0U; i < dlc; ++i) {
                data[i] = can.CAN_RxMsg.Data[i];
            }
            can.rxFrameFlag = false;
            __enable_irq();

            if (((extId >> 8) & 0xFFU) == TEST_MOTOR_ID) {
                received = 1U;
                break;
            }
        }
        delay_ms(1U);
    }

    if (received) {
        serialSendString("CAN RX motor 5: ExtId=0x");
        serialSendHex32(extId);
        serialSendString(" DLC=0x");
        serialSendHex8(dlc);
        serialSendString(" DATA=");
        for (i = 0U; i < dlc; ++i) {
            serialSendHex8(data[i]);
            serialSendChar(' ');
        }
        serialSendString("\r\n");
    } else {
        serialSendString("ERR: motor 5 no CAN reply; ESR=0x");
        serialSendHex32(CAN1->ESR);
        serialSendString(" TSR=0x");
        serialSendHex32(CAN1->TSR);
        serialSendString("\r\n");
    }
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
    } else if (strcmp(command, "cancheck5") == 0) {
        checkMotor5Can();
    } else if (strncmp(command, "servo ", 6U) == 0) {
        processServoCommand(command);
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
        stopServoMotion();
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
    serialSendString("\r\nYYB mecanum/servo jog ready; motors 1..5 stopped; disarmed.\r\n");
    printHelp();

    for (;;) {
        if (emergencyStop) {
            stopServoMotion();
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
        reportCompletedServoMoves();
    }
}
