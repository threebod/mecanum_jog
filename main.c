#include "stm32f4xx.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"

#include <string.h>
#include "straight_control.h"
#include "route_plan.h"

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
extern volatile uint8_t jogCanFault;
static volatile uint32_t clockMs;
static volatile float imuYaw;
static volatile uint32_t imuStamp;
static volatile uint8_t imuValid;
static uint8_t imuFrame[11], imuLength;
static uint8_t motorInvert[5];
static uint16_t motorTrim[5] = {1000U, 1000U, 1000U, 1000U, 1000U};
static int8_t yawSign = 1;
/* 0 idle, 1 finite position test window, 2 IMU heading hold. */
static uint8_t motionMode;
static uint32_t motionStart, motionDuration, lastControl;
static uint16_t straightRpm;
static int16_t straightDirection;
static float targetYaw;
static void serviceMotion(void);
/* Route progress is nominal, NOT measured ground position. */
static uint8_t routeActive, routeWaiting, routeIndex, routeStartZone, routeStep;
static float routeX, routeY, routeYaw;
static int16_t routeForward, routeRight;
static uint32_t routeTick, routeLegStart, routeSettle;
static uint8_t routeAuto, routeRotating, routeOnlyTurn, routeTurnInBand;
static int8_t routeHeading, routeNextHeading;
static float routeBaseYaw;
static uint32_t routeTurnStart, routeTurnStable;
static float routeDriveRpm, routeTurnRpm, routeCorrection;
static int16_t routeSent[4];
static uint8_t routeSentValid;
static uint8_t straightLateral;
static float straightSpeedState, straightTurnState;
static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn);

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

    motionMode = 0U;
    routeActive = 0U;
    routeWaiting = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeSentValid = 0U;
    straightSpeedState = straightTurnState = 0.0f;
    routeForward = routeRight = 0;
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
    serialSendString("  cancheck N query motor 1..5 CAN status\r\n");
    serialSendString("  servo N A set servo 2..4 to angle A\r\n");
    serialSendString("  W/S/A/D   forward/back/left/right\r\n");
    serialSendString("  X, stop   stop motors/servo motion and disarm\r\n");
    serialSendString("  !         emergency stop motors/servo motion\r\n");
    serialSendString("  help      show this help\r\n");
    serialSendString("  line W 100 30   synced forward 100mm at 30rpm (arm)\r\n");
    serialSendString("  straight W 2000 30  IMU heading hold, 2000ms (arm)\r\n");
    serialSendString("  line: W/S; straight: W/S/A/D; max 500mm/5000ms, 10..60rpm\r\n");
    serialSendString("  wheel 1 0      single wheel 1..4, raw dir 0/1 (arm)\r\n");
    serialSendString("  invert 1 1     reverse wheel 1..4 mapping, 0/1\r\n");
    serialSendString("  trim 1 1000    wheel scale 900..1100, RAM only\r\n");
    serialSendString("  imu 115200     IMU baud 9600/115200, PD5 TX / PD6 RX\r\n");
    serialSendString("  yawdir 0       correction sign: 0 normal, 1 reversed\r\n");
    serialSendString("  status         yaw, age, wheel mapping and trims\r\n");
    serialSendString("  route start 1|2  enable and run map route (arm, nose UP)\r\n");
    serialSendString("  route step 1|2   pause at EVERY waypoint (arm)\r\n");
    serialSendString("  route next       continue from a stopped checkpoint\r\n");
    serialSendString("  route status     estimated position / checkpoint\r\n");
    serialSendString("  route auto 1|2   full route with turns; NO station waits (arm)\r\n");
    serialSendString("  turn L|R 1..180  relative IMU turn in clear space (arm)\r\n");
    serialSendString("  Route is a clear-floor test; no obstacle/QR/grasp detection.\r\n");
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
    return emergencyStop != 0U || jogCanFault != 0U;
}

void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        ++clockMs;
    }
}

static void clockInit(void)
{
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 84U - 1U;
    timer.TIM_Period = 1000U - 1U;
    TIM_TimeBaseInit(TIM7, &timer);
    nvic.NVIC_IRQChannel = TIM7_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0U;
    nvic.NVIC_IRQChannelSubPriority = 0U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);
}

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

static void imuInit(uint32_t baud)
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

static void serviceMotion(void)
{
    uint32_t elapsed, age, dt, edge;
    float yaw, error, t;
    int16_t correction;
    uint16_t speed;
    if (!motionMode || emergencyStop) return;
    elapsed = clockMs - motionStart;
    if (elapsed >= motionDuration) {
        stopAllMotors();
        serialSendString("STOP: test window elapsed (not measured distance)\r\n");
        return;
    }
    if (motionMode != 2U || clockMs - lastControl < 20U) return;
    dt = clockMs - lastControl;
    lastControl = clockMs;
    __disable_irq();
    yaw = imuYaw;
    age = clockMs - imuStamp;
    __enable_irq();
    error = headingError(targetYaw, yaw);
    if (!imuValid || age > 250U || dt > 150U || error > 20.0f || error < -20.0f) {
        stopAllMotors();
        serialSendString("ERR: IMU stale or yaw deviation >20deg; stopped\r\n");
        return;
    }
    edge = elapsed < motionDuration - elapsed ? elapsed : motionDuration - elapsed;
    t = edge < 800U ? edge / 800.0f : 1.0f;
    straightSpeedState = routeSlew(straightSpeedState,
        straightRpm * t * t * (3.0f - 2.0f * t), ROUTE_ACCEL_RPM_S, dt);
    speed = (uint16_t)routeRound(straightSpeedState);
    correction = routeAbs(error) < 0.6f ? 0 : headingCorrection(error, yawSign);
    straightTurnState = routeSlew(straightTurnState,
        (float)correction * speed / straightRpm, 30.0f, dt);
    correction = routeRound(straightTurnState);
    sendRouteSpeeds(straightLateral ? 0 : (int16_t)(straightDirection * speed),
        straightLateral ? (int16_t)(straightDirection * speed) : 0, correction);
}

static uint8_t parsePair(const char *cursor, uint16_t *a, uint16_t *b)
{
    if (!parseUint(&cursor, a) || *cursor != ' ' || !parseUint(&cursor, b)) return 0U;
    while (*cursor == ' ') ++cursor;
    return *cursor == '\0';
}

static void startLine(const char *cursor, uint8_t heading)
{
    uint16_t amount, rpm;
    uint8_t id, direction;
    char way = *cursor++;
    uint32_t pulses;
    if ((way != 'W' && way != 'S' && (!heading || (way != 'A' && way != 'D'))) || *cursor != ' ' ||
        !parsePair(cursor, &amount, &rpm) || rpm < 10U || rpm > 60U ||
        (heading && (amount < 1000U || amount > 5000U)) ||
        (!heading && (amount < 20U || amount > 500U))) {
        serialSendString("ERR: line W|S 20..500 10..60; straight W|S|A|D 1000..5000 10..60\r\n");
        return;
    }
    if (!armed) { serialSendString("ERR: send 'arm' first\r\n"); return; }
    armed = 0U;
    if (heading && (!imuValid || clockMs - imuStamp > 250U)) {
        serialSendString("ERR: fresh IMU yaw required; check baud/wiring/status\r\n");
        return;
    }
    if (heading) {
        targetYaw = imuYaw; /* Hold the current heading; no sensor zero command. */
        straightRpm = rpm;
        straightDirection = (way == 'W' || way == 'D') ? 1 : -1;
        straightLateral = (uint8_t)(way == 'A' || way == 'D');
        straightSpeedState = straightTurnState = 0.0f;
        routeSentValid = 0U;
        motionDuration = amount;
        motionStart = clockMs;
        lastControl = clockMs - 20U;
        motionMode = 2U;
    } else {
        for (id = 1U; id <= 4U; ++id) {
            if (motionInterrupted()) return;
            /* Forward axis is the reference project's Y pulse axis. */
            pulses = (uint32_t)amount * 103U * motorTrim[id] / 10000U;
            direction = motorDirections[way == 'W' ? DIRECTION_FORWARD : DIRECTION_BACK][id];
            Emm_V5_Pos_Control(id, direction ^ motorInvert[id],
                (uint16_t)((uint32_t)rpm * motorTrim[id] / 1000U),
                MOTOR_TEST_ACCEL, pulses, false, true);
        }
        if (motionInterrupted()) return;
        Emm_V5_Synchronous_motion(0x00);
        /* 3200 pulses/rev assumed; conservative settling window then explicit stop. */
        motionDuration = ((uint32_t)amount * 103U * 6000U / (3200U * rpm)) + 3000U;
        motionStart = clockMs;
        motionMode = 1U;
    }
    serialSendString(heading ? "RUN heading hold; auto-disarmed\r\n" :
                              "TX queued: distance test; auto-disarmed\r\n");
}

static void printStatus(void)
{
    uint8_t id;
    float yaw = imuYaw;
    uint32_t age = clockMs - imuStamp;
    serialSendString(imuValid && age <= 250U ? "IMU fresh yaw=" : "IMU unavailable/stale yaw=");
    if (yaw < 0.0f) { serialSendChar('-'); yaw = -yaw; }
    serialSendUint((uint16_t)(yaw * 10.0f));
    serialSendString(" (0.1deg), age_ms=");
    serialSendUint((uint16_t)(age > 65535U ? 65535U : age));
    serialSendString(yawSign > 0 ? " yawdir=0\r\n" : " yawdir=1\r\n");
    for (id = 1U; id <= 4U; ++id) {
        serialSendString("wheel="); serialSendUint(id);
        serialSendString(" forward_dir=");
        serialSendUint(motorDirections[0][id] ^ motorInvert[id]);
        serialSendString(" trim="); serialSendUint(motorTrim[id]);
        serialSendString("\r\n");
    }
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
        Emm_V5_Pos_Control(id, motorDirections[(uint8_t)direction][id] ^ motorInvert[id],
                           MOTOR_TEST_SPEED, MOTOR_TEST_ACCEL,
                           MOTOR_TEST_PULSES, false, true);
        delay_ms(2U);
    }

    if (motionInterrupted()) {
        return;
    }
    Emm_V5_Synchronous_motion(0x00);
    motionMode = 1U;
    motionStart = clockMs;
    motionDuration = 2000U;
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

static void checkMotorCan(uint8_t motorId)
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
    Emm_V5_Read_Sys_Params(motorId, S_FLAG);

    for (elapsed = 0U; elapsed < CAN_CHECK_TIMEOUT_MS; ++elapsed) {
        if (motionInterrupted()) {
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

            if (((extId >> 8) & 0xFFU) == motorId && dlc >= 3U &&
                data[0] == 0x3AU && data[dlc - 1U] == 0x6BU) {
                received = 1U;
                break;
            }
        }
        delay_ms(1U);
    }

    if (received) {
        serialSendString("CAN RX motor ");
        serialSendUint(motorId);
        serialSendString(": ExtId=0x");
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
        serialSendString("ERR: motor ");
        serialSendUint(motorId);
        serialSendString(" no CAN reply; ESR=0x");
        serialSendHex32(CAN1->ESR);
        serialSendString(" TSR=0x");
        serialSendHex32(CAN1->TSR);
        serialSendString("\r\n");
    }
}

static void printRouteStatus(void)
{
    serialSendString(routeActive ? (routeWaiting ? "ROUTE WAIT " : "ROUTE RUN ") : "ROUTE IDLE ");
    serialSendString("point="); serialSendUint((uint16_t)(routeIndex + 1U));
    serialSendString(" estimated_x_mm="); serialSendUint((uint16_t)(routeX < 0 ? 0 : routeX));
    serialSendString(" estimated_y_mm="); serialSendUint((uint16_t)(routeY < 0 ? 0 : routeY));
    serialSendString(" (command estimate, NOT localization)\r\n");
}

static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn)
{
    uint8_t id, direction;
    uint8_t changed = (uint8_t)!routeSentValid;
    int16_t speed, speeds[4];
    for (id = 1U; id <= 4U; ++id) {
        speeds[id - 1U] = routeWheel(id, forward, right, turn, motorTrim[id]);
        if (speeds[id - 1U] != routeSent[id - 1U]) changed = 1U;
    }
    /* A velocity command persists in the drive. Re-send the synchronized batch
     * only when a wheel target changes, not every iteration at constant speed. */
    if (!changed) return;
    for (id = 1U; id <= 4U; ++id) {
        if (motionInterrupted()) return;
        speed = speeds[id - 1U];
        direction = motorDirections[0][id] ^ motorInvert[id];
        if (speed < 0) { speed = -speed; direction ^= 1U; }
        Emm_V5_Vel_Control(id, direction, (uint16_t)speed, 0U, true);
    }
    if (!motionInterrupted()) Emm_V5_Synchronous_motion(0x00);
    if (!motionInterrupted()) {
        for (id = 0U; id < 4U; ++id) routeSent[id] = speeds[id];
        routeSentValid = 1U;
    }
}

static void serviceRoute(void)
{
    uint32_t now, dt, age;
    RoutePoint p;
    float yaw, error, dx, dy, remaining;
    int16_t speed, turn, bodyForward, bodyRight;
    uint8_t lateral;
    if (!routeActive || motionInterrupted()) return;
    now = clockMs;
    if (routeWaiting) { routeTick = now; return; }
    dt = now - routeTick;
    if (dt < 20U) return;
    __disable_irq();
    yaw = imuYaw; age = clockMs - imuStamp;
    __enable_irq();
    error = headingError(routeYaw, yaw);
    if (!imuValid || age > 250U || (!routeRotating && routeAbs(error) > 20.0f) || dt > 150U ||
        now - routeLegStart > 45000U) {
        stopAllMotors();
        serialSendString("ERR ROUTE: IMU/deviation/control delay/leg timeout; aborted\r\n");
        return;
    }
    routeTick = now;
    if (routeRotating) {
        if (now - routeTurnStart > 18000U) {
            stopAllMotors(); serialSendString("ERR TURN: timeout; aborted\r\n"); return;
        }
        /* Separate enter/exit thresholds prevent stop/start chatter at 2deg. */
        if (!routeTurnInBand && routeAbs(error) <= 2.0f) routeTurnInBand = 1U;
        if (routeTurnInBand && routeAbs(error) > 3.5f) routeTurnInBand = 0U;
        routeTurnRpm = routeSlew(routeTurnRpm,
            routeTurnInBand ? 0.0f : (float)routeTurnSpeed(error, yawSign), ROUTE_TURN_ACCEL, dt);
        sendRouteSpeeds(0, 0, routeRound(routeTurnRpm));
        if (routeTurnInBand && routeRound(routeTurnRpm) == 0) {
            if (routeTurnInBand == 1U) { routeTurnStable = now; routeTurnInBand = 2U; }
            if (now - routeTurnStable >= 200U) {
                routeRotating = routeTurnInBand = 0U;
                routeHeading = routeNextHeading;
                routeLegStart = now;
                if (routeOnlyTurn) {
                    stopAllMotors(); serialSendString("TURN DONE (2deg entry / 3.5deg hysteresis)\r\n");
                }
            }
        }
        return;
    }
    /* Integration only estimates travel. IMU constrains yaw, not XY drift. */
    routeX += routeEstimate(routeRight, dt, (uint8_t)(routeHeading == 0 || routeHeading == 2));
    routeY += routeEstimate(routeForward, dt, (uint8_t)(routeHeading == 1 || routeHeading == -1));
    if (!routeAuto && now - routeSettle < 300U) return;
    p = routePoint(routeIndex, routeStartZone);
    dx = p.x - routeX; dy = p.y - routeY;
    /* At the endpoint don't reverse just because one sampled step crossed it. */
    if ((routeRight > 0 && dx < 0) || (routeRight < 0 && dx > 0)) {
        routeX = p.x; dx = 0;
    }
    if ((routeForward > 0 && dy < 0) || (routeForward < 0 && dy > 0)) {
        routeY = p.y; dy = 0;
    }
    if (routeAbs(dx) <= 1.5f && routeAbs(dy) <= 1.5f) {
        sendRouteSpeeds(0, 0, 0);
        if (motionInterrupted()) return;
        routeForward = routeRight = 0;
        routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
        routeX = p.x; routeY = p.y; /* snap NOMINAL coordinates only */
        if (p.heading != 4 && p.heading != routeHeading) {
            routeNextHeading = p.heading;
            routeYaw = routeBaseYaw + p.heading * 90.0f * yawSign;
            routeRotating = 1U; routeTurnInBand = 0U;
            routeTurnStart = routeLegStart = clockMs;
            serialSendString("ROUTE turning to station heading\r\n");
            return;
        }
        serialSendString("ROUTE estimated waypoint: ");
        serialSendUint((uint16_t)(routeIndex + 1U));
        if (p.event) { serialSendChar(' '); serialSendString(p.event); }
        serialSendString("\r\n");
        if (routeIndex + 1U == ROUTE_COUNT) {
            stopAllMotors();
            serialSendString("ROUTE END: verify actual home position\r\n");
            return;
        }
        ++routeIndex;
        routeWaiting = (uint8_t)(!routeAuto && (routeStep || p.event != 0));
        routeSettle = routeLegStart = clockMs;
        if (routeWaiting) serialSendString("WAIT: verify position / finish station work; send route next\r\n");
        return;
    }
    /* Map-axis translation is transformed to the current body heading. */
    lateral = routeAbs(dx) > 1.5f;
    remaining = lateral ? routeAbs(dx) : routeAbs(dy);
    speed = routeSpeed(remaining, now - routeLegStart);
    routeDriveRpm = routeSlew(routeDriveRpm, speed, ROUTE_ACCEL_RPM_S, dt);
    speed = routeRound(routeDriveRpm);
    routeRight = lateral ? (dx > 0 ? speed : -speed) : 0;
    routeForward = lateral ? 0 : (dy > 0 ? speed : -speed);
    turn = routeAbs(error) < 0.6f ? 0 : headingCorrection(error, yawSign);
    routeCorrection = routeSlew(routeCorrection,
        (float)turn * speed / ROUTE_RPM, 30.0f, dt);
    turn = routeRound(routeCorrection);
    routeBody(routeForward, routeRight, routeHeading, &bodyForward, &bodyRight);
    sendRouteSpeeds(bodyForward, bodyRight, turn);
}

static uint8_t processRouteCommand(const char *command)
{
    uint8_t start, step, autoRun;
    uint16_t degrees;
    const char *cursor;
    if (strcmp(command, "route status") == 0) { printRouteStatus(); return 1U; }
    if (strncmp(command, "turn ", 5U) == 0) {
        cursor = command + 7;
        if (strlen(command) < 8U || (command[5] != 'L' && command[5] != 'R') ||
            command[6] != ' ' || !parseUint(&cursor, &degrees) || *cursor != '\0' ||
            degrees < 1U || degrees > 180U) {
            serialSendString("ERR: turn L|R 1..180\r\n"); return 1U;
        }
        if (motionMode || routeActive || !armed || !imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
            serialSendString("ERR: turn requires idle, arm, fresh IMU and healthy CAN\r\n"); return 1U;
        }
        armed = 0U;
        routeYaw = imuYaw + (command[5] == 'L' ? degrees : -(float)degrees) * yawSign;
        setAllMotorsEnabled(true);
        if (motionInterrupted()) { stopAllMotors(); return 1U; }
        routeForward = routeRight = 0;
        routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
        routeSentValid = 0U;
        routeTick = routeTurnStart = routeLegStart = clockMs;
        routeWaiting = routeTurnInBand = 0U;
        routeOnlyTurn = routeRotating = routeActive = 1U;
        serialSendString("TURN running\r\n"); return 1U;
    }
    if (strcmp(command, "route next") == 0) {
        if (!routeActive || !routeWaiting) {
            serialSendString("ERR: no waiting route\r\n"); return 1U;
        }
        if (!imuValid || clockMs - imuStamp > 250U ||
            routeAbs(headingError(routeYaw, imuYaw)) > 20.0f) {
            serialSendString("ERR: restore fresh IMU / original heading first\r\n"); return 1U;
        }
        routeWaiting = 0U;
        routeTick = routeSettle = routeLegStart = clockMs;
        serialSendString("ROUTE continuing\r\n"); return 1U;
    }
    if (strcmp(command, "route start 1") != 0 && strcmp(command, "route start 2") != 0 &&
        strcmp(command, "route step 1") != 0 && strcmp(command, "route step 2") != 0 &&
        strcmp(command, "route auto 1") != 0 && strcmp(command, "route auto 2") != 0) return 0U;
    if (motionMode || routeActive) { serialSendString("ERR: busy; stop first\r\n"); return 1U; }
    if (!armed) { serialSendString("ERR: send 'arm' first\r\n"); return 1U; }
    armed = 0U;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        serialSendString("ERR: fresh IMU and healthy CAN required\r\n"); return 1U;
    }
    step = (uint8_t)(strncmp(command, "route step ", 11U) == 0);
    autoRun = (uint8_t)(strncmp(command, "route auto ", 11U) == 0);
    start = (uint8_t)(command[strlen(command) - 1U] - '0');
    routeAuto = autoRun; routeHeading = routeNextHeading = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeSentValid = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeStartZone = start; routeStep = step; routeIndex = 0U;
    routeX = 2250.0f; routeY = start == 1U ? 2250.0f : 150.0f;
    routeBaseYaw = routeYaw = imuYaw;
    routeForward = routeRight = 0;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) { stopAllMotors(); return 1U; }
    routeTick = routeSettle = routeLegStart = clockMs;
    routeWaiting = 0U; routeActive = 1U;
    serialSendString("ROUTE START: nose UP, clear floor, nominal distance only\r\n");
    return 1U;
}

static void processCommand(const char *command)
{
    uint16_t id, value;
    if (emergencyStop) return;
    if (processRouteCommand(command)) return;
    if (routeActive && strcmp(command, "stop") != 0 && strcmp(command, "X") != 0 &&
        strcmp(command, "x") != 0 && strcmp(command, "disable") != 0 &&
        !(routeWaiting && strncmp(command, "servo ", 6U) == 0)) {
        serialSendString("ERR: route active; use route status/next or stop/!\r\n");
        return;
    }
    if (motionMode && strcmp(command, "stop") != 0 && strcmp(command, "X") != 0 &&
        strcmp(command, "x") != 0 && strcmp(command, "disable") != 0) {
        serialSendString("ERR: test busy; use stop or ! first\r\n");
        return;
    }
    if (strncmp(command, "line ", 5U) == 0) {
        startLine(command + 5, 0U);
        return;
    }
    if (strncmp(command, "straight ", 9U) == 0) {
        startLine(command + 9, 1U);
        return;
    }
    if (strcmp(command, "status") == 0) { printStatus(); return; }
    if (strncmp(command, "cancheck ", 9U) == 0) {
        const char *cursor = command + 9;
        if (!parseUint(&cursor, &id) || *cursor != '\0' || id < 1U || id > 5U) {
            serialSendString("ERR: cancheck 1..5\r\n");
        } else checkMotorCan((uint8_t)id);
        return;
    }
    if (strcmp(command, "imu 9600") == 0 || strcmp(command, "imu 115200") == 0) {
        imuInit(strcmp(command, "imu 9600") == 0 ? 9600U : 115200U);
        serialSendString("IMU receiver configured; sensor settings unchanged\r\n");
        return;
    }
    if (strcmp(command, "yawdir 0") == 0 || strcmp(command, "yawdir 1") == 0) {
        yawSign = command[7] == '0' ? 1 : -1;
        printStatus();
        return;
    }
    if (strncmp(command, "invert ", 7U) == 0 || strncmp(command, "trim ", 5U) == 0 ||
        strncmp(command, "wheel ", 6U) == 0) {
        uint8_t invert = command[0] == 'i';
        uint8_t wheel = command[0] == 'w';
        if (!parsePair(command + (invert ? 7 : wheel ? 6 : 5), &id, &value) ||
            id < 1U || id > 4U || ((invert || wheel) && value > 1U) ||
            (!invert && !wheel && (value < 900U || value > 1100U))) {
            serialSendString("ERR: wheel/invert ID(1..4) 0|1; trim ID 900..1100\r\n");
            return;
        }
        if (wheel) {
            if (!armed) { serialSendString("ERR: send 'arm' first\r\n"); return; }
            armed = 0U;
            Emm_V5_Pos_Control((uint8_t)id, (uint8_t)value, MOTOR_TEST_SPEED,
                              MOTOR_TEST_ACCEL, MOTOR_TEST_PULSES, false, false);
            motionMode = 1U;
            motionStart = clockMs;
            motionDuration = 2000U;
            serialSendString("TX queued: raw single wheel jog\r\n");
        } else {
            if (invert) motorInvert[id] = (uint8_t)value;
            else motorTrim[id] = value;
            armed = 0U;
            printStatus();
        }
        return;
    }
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
        stopAllMotors();
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
        checkMotorCan(TEST_MOTOR_ID);
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
    static uint8_t discardLine;

    if (USART_GetITStatus(UART5, USART_IT_RXNE) == RESET) {
        return;
    }

    received = (char)USART_ReceiveData(UART5);
    if (received == '!') {
        emergencyStop = 1U;
        discardLine = 0U;
        rxLength = 0U;
        rxReady = 0U;
    } else if (discardLine) {
        if (received == '\r' || received == '\n') discardLine = 0U;
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
            discardLine = 1U;
        }
    }

    USART_ClearITPendingBit(UART5, USART_IT_RXNE);
}

int main(void)
{
    char command[RX_LINE_SIZE];

    delay_init(168U);
    board_init();
    clockInit();
    serialInit();
    imuInit(115200U);

    stopAllMotors();
    serialSendString("\r\nYYB mecanum/servo jog ready; motors 1..5 stopped; disarmed.\r\n");
    printHelp();

    for (;;) {
        if (jogCanFault) {
            stopAllMotors();
            jogCanFault = 0U;
            serialSendString("ERR: CAN transmit failed; stop attempted, check power/bus\r\n");
        }
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
        serviceMotion();
        serviceRoute();
        reportCompletedServoMoves();
    }
}
