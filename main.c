#include "stm32f4xx.h"
#include "delay.h"
#include "board.h"
#include "Emm_V5.h"

#include <string.h>
#include "straight_control.h"
#include "route_plan.h"
#include "map_navigation.h"
#include "mechanism_action.h"
#include "raw_pick_action.h"
#include "mission_actions.h"
#include "vision_control.h"
#include "vision_config.h"
#include "hwt101.h"
#include "jog_can.h"

#define COMMAND_BAUD_RATE  115200U
#define RX_LINE_SIZE       80U

#define MOTOR_MIN_ID       1U
#define MOTOR_MAX_ID       4U
#define AUX_MOTOR_MIN_ID   5U
#define AUX_MOTOR_MAX_ID   6U
#define MOTOR_TEST_SPEED   30U
#define MOTOR_TEST_ACCEL   50U
#define MOTOR_TEST_PULSES  160U
#define AUX_MOVE_STATUS_POLL_MS  50U
#define AUX_MOVE_TIMEOUT_MARGIN_MS  3000U
#define HOST_HEARTBEAT_TIMEOUT_MS  1000U

#define SERVO_MIN_PULSE_US    500U
#define SERVO_PULSE_RANGE_US  2000U
#define SERVO_PERIOD_US       20000U
#define SERVO_DEFAULT_SPEED_DPS10 1200U
#define SERVO_MAX_SPEED_DPS10     1800U
#define SERVO_UPDATE_HZ       50U
#define SERVO_STEP_MDEG       ((SERVO_DEFAULT_SPEED_DPS10 * 100U) / SERVO_UPDATE_HZ)

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
    COMMAND_ANGLE_ERROR,
    COMMAND_SPEED_ERROR
} CommandResult;

typedef struct {
    uint8_t channel;
    uint16_t angle;
    uint16_t speedDps10;
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
static volatile uint32_t servoStepMdeg[3] = {
    SERVO_STEP_MDEG, SERVO_STEP_MDEG, SERVO_STEP_MDEG
};
extern volatile uint8_t jogCanFault;
volatile uint32_t clockMs;
static uint8_t hostHeartbeatActive;
static uint32_t hostHeartbeatStamp;
static uint8_t motorInvert[5];
static uint16_t motorTrim[5] = {1000U, 1000U, 1000U, 1000U, 1000U};
static int8_t yawSign = 1;
/* 0 idle, 1 finite position test window, 2 IMU heading hold, 3 aux position. */
static uint8_t motionMode;
static uint32_t motionStart, motionDuration, lastControl;
static uint8_t auxMoveMotorId;
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
static uint8_t fullRouteRunning;
static uint8_t rawPickRouteActive, rawPickAtStation, rawPickItemIndex;
static uint8_t rawPickMissingReported;
static MechanismAction rawPickRouteActions[RAW_PICK_ACTION_COUNT];
static const uint8_t rawPickColors[3] = {4U, 2U, 6U};
typedef enum {
    MISSION_IDLE, MISSION_QR_WAIT, MISSION_RAW_PICK,
    MISSION_RING_ALIGN, MISSION_TEMP_ALIGN, MISSION_COARSE_PLACE, MISSION_COARSE_PICK,
    MISSION_TEMP_PLACE, MISSION_RECENTER
} MissionPhase;
static uint8_t missionActive, missionActionStarted;
static MissionPhase missionPhase;
static uint32_t missionDeadline;
static uint8_t missionObservedItem;
static float missionOffsetForward, missionOffsetRight;
static MechanismAction missionWorkActions[MISSION_PLACE_COUNT];
static MechanismAction missionPickBackActions[MISSION_PICK_BACK_COUNT];
static const MechanismPose missionRingObservePose =
    {-500, 0, 2700, 30, 50, 90, 50, 1200};
static int8_t routeHeading, routeNextHeading;
static float routeBaseYaw;
static uint32_t routeTurnStart, routeTurnStable;
static uint8_t navInitialized, navRunning, navCurrentNode;
static NavPoint navPath[NAV_PATH_CAPACITY];
static uint8_t navPathCount;
static uint32_t navLastReport, routeLastReport;
static float routeDriveRpm, routeTurnRpm, routeCorrection;
static uint32_t headingPidLastReport;
static RouteHeadingPid routeHeadingPid;
static RouteHeadingPidGains routeHeadingGains = {
    ROUTE_HEADING_KP, ROUTE_HEADING_KI, ROUTE_HEADING_KD
};
static uint16_t routeRpm = ROUTE_RPM;
static float routeForwardScale = ROUTE_FORWARD_SCALE;
static float routeLateralScale = ROUTE_LATERAL_SCALE;
static uint16_t routeTurnRpmLimit = (uint16_t)ROUTE_TURN_RPM;
static uint16_t routeLateralRpmLimit = ROUTE_LATERAL_RPM_MAX;
static int16_t routeSent[4];
static uint8_t routeSentValid;
static uint8_t straightLateral;
static uint8_t straightUseRoutePid;
static float straightSpeedState, straightTurnState;
static void sendRouteSpeeds(int16_t forward, int16_t right, int16_t turn);
static void printHeadingPidTrace(float targetYaw, float actualYaw, int16_t turn);
static void serialSendString(const char *text);
static MechanismState mechanismState;
static const MechanismInitialState *mechanismActionInitial;
static const MechanismAction *mechanismActions;
static uint16_t mechanismActionCount, mechanismActionIndex;
static uint8_t mechanismActionActive, mechanismActionDispatched;
static uint32_t mechanismActionWaitUntil;
static VisionSession visionSession;
static VisionParser visionParser;
static volatile VisionPacket visionRxPacket;
static volatile uint8_t visionRxReady;
static uint8_t visionMotionAutomatic;
static uint8_t visionPickActive;
static const VisionCalibration visionPickCalibration = {
    1U, {0, 0, 320, 240}, 160, 120,
    {0.0f, 0.0f, 0.0f, 0.0f}
};

static void serviceVision(void);
static void serviceRawPickRoute(void);
static void serviceMission(void);
static void missionBeginStation(uint8_t nextIndex);
static uint8_t processVisionCommand(const char *command);
static uint8_t visionSessionActive(void);
static void failVision(const char *reason);

static void invalidateMechanism(const char *reason)
{
    if (!mechanismState.valid && !mechanismState.running) return;
    mechanismStateInvalidate(&mechanismState);
    serialSendString("MECH INVALID reason=");
    serialSendString(reason);
    serialSendString("\r\n");
}

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

static void serialSendInt(int16_t value)
{
    if (value < 0) {
        serialSendChar('-');
        value = (int16_t)-value;
    }
    serialSendUint((uint16_t)value);
}

static void invalidateNavigation(const char *reason)
{
    if (!navInitialized) return;
    navInitialized = 0U;
    navRunning = 0U;
    navPathCount = 0U;
    serialSendString("NAV INVALID reason=");
    serialSendString(reason);
    serialSendString("\r\n");
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

static void serialSendUint32(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);

    while (count > 0U) {
        serialSendChar(digits[--count]);
    }
}

static void visionUartInit(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    USART_StructInit(&uart);
    uart.USART_BaudRate = 9600U;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART4, &uart);
    nvic.NVIC_IRQChannel = UART4_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1U;
    nvic.NVIC_IRQChannelSubPriority = 1U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    vision_parser_init(&visionParser);
    visionRxReady = 0U;
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART4, ENABLE);
}

static void visionUartSend(const VisionPacket *packet)
{
    uint8_t frame[VISION_FRAME_SIZE];
    uint8_t index;
    vision_packet_encode(packet, frame);
    for (index = 0U; index < VISION_FRAME_SIZE; ++index) {
        while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(UART4, frame[index]);
    }
}

void UART4_IRQHandler(void)
{
    uint8_t byte;
    VisionPacket packet;
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == RESET) return;
    byte = (uint8_t)USART_ReceiveData(UART4);
    if (!visionRxReady && vision_parser_feed(&visionParser, &byte, 1U, &packet)) {
        visionRxPacket = packet;
        visionRxReady = 1U;
    }
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
}

static void stopDriveMotors(void)
{
    uint8_t id;

    for (id = MOTOR_MIN_ID; id <= MOTOR_MAX_ID; ++id) {
        Emm_V5_Stop_Now(id, false);
        delay_ms(2U);
    }
}

static void stopAuxMotors(void)
{
    uint8_t id;

    for (id = AUX_MOTOR_MIN_ID; id <= AUX_MOTOR_MAX_ID; ++id) {
        Emm_V5_Stop_Now(id, false);
        delay_ms(2U);
    }
}

static void stopAllMotors(void)
{
    if (visionSession.state == VISION_STATE_REQUEST ||
        visionSession.state == VISION_STATE_WAIT ||
        visionSession.state == VISION_STATE_MOVE ||
        visionSession.state == VISION_STATE_SETTLE ||
        visionSession.state == VISION_STATE_PREP ||
        visionSession.state == VISION_STATE_PICK || motionMode == 4U) {
        vision_session_pause(&visionSession);
    }
    visionPickActive = 0U;
    invalidateMechanism("stopped");
    mechanismActionActive = 0U;
    motionMode = 0U;
    straightUseRoutePid = 0U;
    routeActive = 0U;
    fullRouteRunning = 0U;
    missionActive = missionActionStarted = 0U;
    missionPhase = MISSION_IDLE;
    missionOffsetForward = missionOffsetRight = 0.0f;
    rawPickRouteActive = rawPickAtStation = rawPickItemIndex = 0U;
    rawPickMissingReported = 0U;
    routeWaiting = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = 0U;
    straightSpeedState = straightTurnState = 0.0f;
    routeForward = routeRight = 0;
    navInitialized = 0U;
    navRunning = 0U;
    navPathCount = 0U;
    stopDriveMotors();
    stopAuxMotors();
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

static void setAuxMotorsEnabled(bool enabled)
{
    uint8_t id;

    for (id = AUX_MOTOR_MIN_ID; id <= AUX_MOTOR_MAX_ID; ++id) {
        Emm_V5_En_Control(id, enabled, false);
        delay_ms(2U);
    }
}

static void printHelp(void)
{
    serialSendString("\r\nMecanum jog commands:\r\n");
    serialSendString("  arm       authorize ONE motion command\r\n");
    serialSendString("  disable   stop/disable motors 1..6 and disarm\r\n");
    serialSendString("  motor N D jog motor 5|6 in direction 0|1 (armed)\r\n");
    serialSendString("  auxmove N D R A move motor 5|6 by signed 0.1mm (armed)\r\n");
    serialSendString("  cancheck N query motor 1..6 CAN status\r\n");
    serialSendString("  servo N A [S] set angle; speed S=10..1800 x0.1dps\r\n");
    serialSendString("  W/S/A/D   forward/back/left/right\r\n");
    serialSendString("  X, stop   stop motors/servo motion and disarm\r\n");
    serialSendString("  !         emergency stop motors/servo motion\r\n");
    serialSendString("  help      show this help\r\n");
    serialSendString("  line W 100 30   synced forward 100mm at 30rpm (arm)\r\n");
    serialSendString("  straight W 2000 30  IMU heading hold, 2000ms (arm)\r\n");
    serialSendString("  pid move A|D 2000 20  lateral heading PID test, 1000..5000ms, 10..lateral limit rpm (arm)\r\n");
    serialSendString("  line: W/S, 100..500mm step 100; straight: W/S/A/D; 10..120rpm\r\n");
    serialSendString("  wheel 1 0      single wheel 1..4, raw dir 0/1 (arm)\r\n");
    serialSendString("  invert 1 1     reverse wheel 1..4 mapping, 0/1\r\n");
    serialSendString("  trim 1 1000    wheel scale 900..1100, RAM only\r\n");
    serialSendString("  imu 115200     IMU baud 9600/115200, PD5 TX / PD6 RX\r\n");
    serialSendString("  vision align material 1..6 | ring 1..3 (armed)\r\n");
    serialSendString("  vision scale material F R  apply checked mm/pixel x1000 (armed)\r\n");
    serialSendString("  vision pick material 1..6  stationary color-gated pickup (armed)\r\n");
    serialSendString("  vision scale ring N F R  mm/pixel x1000, RAM only (armed)\r\n");
    serialSendString("  vision pause/status | vision jog F R RPM (armed)\r\n");
    serialSendString("  yawdir 0       correction sign: 0 normal, 1 reversed\r\n");
    serialSendString("  status         yaw, age, wheel mapping and trims\r\n");
    serialSendString("  route start 1|2 [rpm]  run map route (arm, nose UP)\r\n");
    serialSendString("  route step 1|2 [rpm]   pause at EVERY waypoint (arm)\r\n");
    serialSendString("  route next       continue from a stopped checkpoint\r\n");
    serialSendString("  route status     estimated position / checkpoint\r\n");
    serialSendString("  route tune F L T [V]  scales, turn and lateral RPM limit, RAM only\r\n");
    serialSendString("  route auto 1|2 [rpm]   full route; rpm 10..120 (arm)\r\n");
    serialSendString("  route rawpick 1|2 [rpm]  stop at RAW_1, pick green/yellow/light blue (arm)\r\n");
    serialSendString("  route mission 1|2 [rpm]  two-round pickup/place mission (arm)\r\n");
    serialSendString("  turn L|R 1..180  relative IMU turn in clear space (arm)\r\n");
    serialSendString("  nav init 1|2       set estimated start pose; fresh IMU, idle\r\n");
    serialSendString("  nav goto X Y [rpm] safe-corridor target; rpm 10..120 (arm)\r\n");
    serialSendString("  nav status         estimated navigation state\r\n");
    serialSendString("  pid get            read navigation heading PID (x100)\r\n");
    serialSendString("  pid set KP KI KD   set RAM gains while stopped (x100)\r\n");
    serialSendString("  mech init H L T     set manual mechanism pose (0.1mm/0.1deg)\r\n");
    serialSendString("  mech pose H L T HR HA LR LA TS  move pose; lift 0..1500 (arm)\r\n");
    serialSendString("  mech status         estimated mechanism state\r\n");
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
    uint16_t speedDps10 = SERVO_DEFAULT_SPEED_DPS10;

    if (strncmp(cursor, "servo ", 6U) != 0) {
        return COMMAND_FORMAT_ERROR;
    }
    cursor += 6;
    if (!parseUint(&cursor, &channel) || !parseUint(&cursor, &angle)) {
        return COMMAND_FORMAT_ERROR;
    }
    while (*cursor == ' ') cursor++;
    if (*cursor != '\0') {
        if (!parseUint(&cursor, &speedDps10)) return COMMAND_FORMAT_ERROR;
        while (*cursor == ' ') cursor++;
        if (*cursor != '\0') return COMMAND_FORMAT_ERROR;
    }
    if (channel < 2U || channel > 4U) {
        return COMMAND_SERVO_ERROR;
    }
    if ((channel < 4U && angle > 270U) ||
        (channel == 4U && angle > 360U)) {
        return COMMAND_ANGLE_ERROR;
    }
    if (speedDps10 < 10U || speedDps10 > SERVO_MAX_SPEED_DPS10) {
        return COMMAND_SPEED_ERROR;
    }

    command->channel = (uint8_t)channel;
    command->angle = angle;
    command->speedDps10 = speedDps10;
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

static uint8_t servoSetTargetMdeg(uint8_t channel, uint32_t target,
                                  uint16_t speedDps10)
{
    uint8_t index = (uint8_t)(channel - 2U);
    uint32_t step = (uint32_t)speedDps10 * 100U / SERVO_UPDATE_HZ;
    if (step == 0U) step = 1U;

    if (!servoChannelEnabled[index]) {
        servoSetPulse(channel, servoAngleToPulse(channel, target));
        __disable_irq();
        currentAngleMdeg[index] = target;
        targetAngleMdeg[index] = target;
        servoStepMdeg[index] = step;
        servoChannelMoving[index] = 0U;
        __enable_irq();
        return 0U;
    }

    __disable_irq();
    targetAngleMdeg[index] = target;
    servoStepMdeg[index] = step;
    servoChannelMoving[index] = currentAngleMdeg[index] != target;
    completedServoChannels &= (uint8_t)~(1U << index);
    __enable_irq();
    return servoChannelMoving[index];
}

static uint8_t servoSetTarget(uint8_t channel, uint16_t angle,
                              uint16_t speedDps10)
{
    return servoSetTargetMdeg(channel, (uint32_t)angle * 1000U,
                              speedDps10);
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

static void serviceHostWatchdog(void)
{
    uint8_t hadNavigation, hadFullRoute, hadVision;
    if (!hostHeartbeatActive ||
        clockMs - hostHeartbeatStamp <= HOST_HEARTBEAT_TIMEOUT_MS) {
        return;
    }

    hostHeartbeatActive = 0U;
    hadNavigation = navInitialized;
    hadFullRoute = fullRouteRunning;
    hadVision = visionSessionActive();
    stopServoMotion();
    stopAllMotors();
    if (hadNavigation) serialSendString("NAV INVALID reason=heartbeat_timeout\r\n");
    if (hadFullRoute) serialSendString("ROUTE INVALID reason=heartbeat_timeout\r\n");
    if (hadVision) failVision("HEARTBEAT_TIMEOUT");
    serialSendString("ERR: host heartbeat timeout; stopped\r\n");
}

static void processServoCommand(const char *commandText)
{
    ServoCommand command;
    CommandResult result = parseServoCommand(commandText, &command);
    uint16_t pulse;
    uint8_t moving;

    if (result == COMMAND_FORMAT_ERROR) {
        serialSendString("ERR format: servo <2|3|4> <angle> [speed_dps10]\r\n");
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
    if (result == COMMAND_SPEED_ERROR) {
        serialSendString("ERR speed: 10..1800 x0.1dps\r\n");
        return;
    }

    pulse = servoAngleToPulse(command.channel,
                              (uint32_t)command.angle * 1000U);
    moving = servoSetTarget(command.channel, command.angle,
                            command.speedDps10);
    serialSendString(moving ? "MOVING servo=" : "OK servo=");
    serialSendUint(command.channel);
    serialSendString(moving ? " target=" : " angle=");
    serialSendUint(command.angle);
    if (moving) {
        serialSendString(" speed_dps10=");
        serialSendUint(command.speedDps10);
        serialSendString("\r\n");
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
            current = target - current <= servoStepMdeg[index] ?
                      target : current + servoStepMdeg[index];
        } else {
            current = current - target <= servoStepMdeg[index] ?
                      target : current - servoStepMdeg[index];
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

static uint8_t visionSessionActive(void)
{
    return visionSession.state == VISION_STATE_REQUEST ||
           visionSession.state == VISION_STATE_WAIT ||
           visionSession.state == VISION_STATE_MOVE ||
           visionSession.state == VISION_STATE_SETTLE ||
           visionSession.state == VISION_STATE_PREP ||
           visionSession.state == VISION_STATE_PICK;
}

static const VisionCalibration *visionCalibration(void)
{
    if (visionPickActive) return rawPickAtStation ?
        &visionMaterialCalibration : &visionPickCalibration;
    if (visionSession.mode == VISION_MODE_MATERIAL) {
        return &visionMaterialCalibration;
    }
    if (visionSession.target >= 1U && visionSession.target <= 3U) {
        return &visionRingCalibration[visionSession.target - 1U];
    }
    return (const VisionCalibration *)0;
}

static const char *visionStateName(VisionState state)
{
    switch (state) {
    case VISION_STATE_REQUEST: return "REQUEST";
    case VISION_STATE_WAIT: return "WAIT";
    case VISION_STATE_MOVE: return "MOVE";
    case VISION_STATE_SETTLE: return "SETTLE";
    case VISION_STATE_ALIGNED: return "ALIGNED";
    case VISION_STATE_FAILED: return "FAILED";
    case VISION_STATE_PAUSED: return "PAUSED";
    case VISION_STATE_PREP: return "PREP";
    case VISION_STATE_PICK: return "PICK";
    case VISION_STATE_PICK_DONE: return "PICK_DONE";
    default: return "IDLE";
    }
}

static void serialSendFloat1(float value)
{
    uint16_t integer;
    uint16_t fraction;
    if (value < 0.0f) {
        serialSendChar('-');
        value = -value;
    }
    integer = (uint16_t)value;
    fraction = (uint16_t)((value - integer) * 10.0f + 0.5f);
    if (fraction >= 10U) {
        ++integer;
        fraction = 0U;
    }
    serialSendUint(integer);
    serialSendChar('.');
    serialSendUint(fraction);
}

static void printVisionState(void)
{
    serialSendString("VISION STATE state=");
    serialSendString(visionStateName(visionSession.state));
    serialSendString(" mode=");
    serialSendString(visionSession.mode == VISION_MODE_RING ? "RING" : "MATERIAL");
    serialSendString(" selector=");
    serialSendUint(visionSession.selector);
    serialSendString(" target=");
    serialSendUint(visionSession.target);
    serialSendString(" iteration=");
    serialSendUint(visionSession.iteration);
    serialSendString("\r\n");
}

static void failVision(const char *reason)
{
    uint8_t wasRawPick = rawPickAtStation;
    uint8_t wasMission = missionActive;
    uint8_t slot = rawPickItemIndex < 3U ?
        (uint8_t)(rawPickItemIndex + 1U) : 3U;
    uint8_t color = wasRawPick ? rawPickColors[slot - 1U] : 0U;
    if (visionPickActive || wasMission) {
        stopServoMotion();
        stopAllMotors();
    }
    if (motionMode == 4U) {
        stopDriveMotors();
        motionMode = 0U;
    }
    visionSession.state = VISION_STATE_FAILED;
    visionSession.fault = reason;
    serialSendString("VISION ERROR reason=");
    serialSendString(reason);
    serialSendString("\r\n");
    if (wasRawPick) {
        serialSendString("ROUTE RAWPICK state=ERROR color=");
        serialSendUint(color);
        serialSendString(" slot=");
        serialSendUint(slot);
        serialSendString("\r\n");
    }
    if (wasMission) serialSendString("ROUTE INVALID reason=mission_vision\r\n");
    else if (wasRawPick) serialSendString("ROUTE INVALID reason=rawpick_vision\r\n");
}

static uint8_t startVisionMotion(float forwardMm, float rightMm,
                                 uint16_t rpm, uint8_t automatic)
{
    float forward = forwardMm / routeForwardScale;
    float right = rightMm / routeLateralScale;
    float wheelMm[4];
    uint32_t maximumPulses = 0U;
    uint8_t id;
    uint8_t moved = 0U;
    wheelMm[0] = forward - right;
    wheelMm[1] = forward + right;
    wheelMm[2] = forward - right;
    wheelMm[3] = forward + right;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) return 0U;
    for (id = 1U; id <= 4U; ++id) {
        float distance = wheelMm[id - 1U];
        uint8_t direction = motorDirections[DIRECTION_FORWARD][id] ^ motorInvert[id];
        uint32_t pulses;
        if (distance < 0.0f) {
            distance = -distance;
            direction ^= 1U;
        }
        pulses = (uint32_t)(distance * 3200.0f / ROUTE_MM_PER_REV + 0.5f);
        if (pulses == 0U) {
            Emm_V5_Stop_Now(id, false);
            continue;
        }
        if (pulses > maximumPulses) maximumPulses = pulses;
        moved = 1U;
        Emm_V5_Pos_Control(id, direction,
                           (uint16_t)((uint32_t)rpm * motorTrim[id] / 1000U),
                           MOTOR_TEST_ACCEL, pulses, false, true);
        delay_ms(2U);
    }
    if (!moved || motionInterrupted()) return 0U;
    Emm_V5_Synchronous_motion(0x00);
    invalidateNavigation("vision_motion");
    motionStart = clockMs;
    motionDuration = maximumPulses * 60000U / (3200U * rpm) + 800U;
    motionMode = 4U;
    visionMotionAutomatic = automatic;
    if (missionActive && routeWaiting) {
        missionOffsetForward += forwardMm;
        missionOffsetRight += rightMm;
    }
    return 1U;
}

static void serviceMotion(void)
{
    uint32_t elapsed, age, dt, edge, sampleMs;
    uint32_t extId;
    uint8_t dlc, function, status, checksum;
    float yaw, error, t, targetCorrection;
    int16_t correction;
    uint16_t speed;
    if (!motionMode || emergencyStop) return;
    elapsed = clockMs - motionStart;
    if (motionMode == 4U) {
        if (motionInterrupted()) {
            failVision("MOTION_FAULT");
            return;
        }
        if (elapsed >= motionDuration) {
            stopDriveMotors();
            motionMode = 0U;
            if (visionMotionAutomatic) {
                vision_session_move_complete(&visionSession, clockMs);
                printVisionState();
            } else {
                vision_session_pause(&visionSession);
                serialSendString("VISION PAUSED\r\n");
            }
        }
        return;
    }
    if (motionMode == 3U) {
        extId = 0U;
        dlc = function = status = checksum = 0U;
        __disable_irq();
        if (can.rxFrameFlag) {
            extId = can.CAN_RxMsg.ExtId;
            dlc = can.CAN_RxMsg.DLC;
            if (dlc >= 3U) {
                function = can.CAN_RxMsg.Data[0];
                status = can.CAN_RxMsg.Data[1];
                checksum = can.CAN_RxMsg.Data[dlc - 1U];
            }
            can.rxFrameFlag = false;
        }
        __enable_irq();
        if (((extId >> 8) & 0xFFU) == auxMoveMotorId && dlc >= 3U &&
            function == 0x3AU && checksum == 0x6BU && (status & 0x02U)) {
            motionMode = 0U;
            serialSendString("DONE: auxmove motor=");
            serialSendUint(auxMoveMotorId);
            serialSendString(" reached target\r\n");
            return;
        }
        if (elapsed >= motionDuration) {
            stopAllMotors();
            serialSendString("ERR: auxmove status timeout; stopped\r\n");
            return;
        }
        if (clockMs - lastControl >= AUX_MOVE_STATUS_POLL_MS) {
            lastControl = clockMs;
            Emm_V5_Read_Sys_Params(auxMoveMotorId, S_FLAG);
        }
        return;
    }
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
    sampleMs = imuStamp;
    age = clockMs - sampleMs;
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
    targetCorrection = straightUseRoutePid ?
        routeHeadingPidStep(&routeHeadingPid, &routeHeadingGains,
                            error, yaw, sampleMs, dt, yawSign) :
        (float)(routeAbs(error) < 0.6f ? 0 : headingCorrection(error, yawSign));
    straightTurnState = routeSlew(straightTurnState,
        targetCorrection * speed / straightRpm,
        straightUseRoutePid ? 90.0f : 30.0f, dt);
    correction = routeRound(straightTurnState);
    sendRouteSpeeds(straightLateral ? 0 : (int16_t)(straightDirection * speed),
        straightLateral ? (int16_t)(straightDirection * speed) : 0, correction);
    if (straightUseRoutePid && !motionInterrupted() &&
        clockMs - headingPidLastReport >= 200U) {
        headingPidLastReport = clockMs;
        printHeadingPidTrace(targetYaw, yaw, correction);
    }
}

static uint8_t parsePair(const char *cursor, uint16_t *a, uint16_t *b)
{
    if (!parseUint(&cursor, a) || *cursor != ' ' || !parseUint(&cursor, b)) return 0U;
    while (*cursor == ' ') ++cursor;
    return *cursor == '\0';
}

static uint8_t parsePairWithOptionalRpm(const char *cursor, uint16_t *a,
                                        uint16_t *b, uint16_t *rpm)
{
    if (!parseUint(&cursor, a) || *cursor != ' ') return 0U;
    ++cursor;
    if (!parseUint(&cursor, b)) return 0U;
    if (*cursor == '\0') {
        *rpm = ROUTE_RPM;
        return 1U;
    }
    if (*cursor != ' ') return 0U;
    ++cursor;
    return parseUint(&cursor, rpm) && *cursor == '\0';
}

static uint8_t parseValueWithOptionalRpm(const char *cursor, uint16_t *value,
                                         uint16_t *rpm)
{
    if (!parseUint(&cursor, value)) return 0U;
    if (*cursor == '\0') {
        *rpm = ROUTE_RPM;
        return 1U;
    }
    if (*cursor != ' ') return 0U;
    ++cursor;
    return parseUint(&cursor, rpm) && *cursor == '\0';
}

static void printMechanismPose(const char *prefix, const MechanismPose *pose)
{
    serialSendString(prefix);
    serialSendString(" h=");
    serialSendInt(pose->horizontalDmm);
    serialSendString(" l=");
    serialSendUint(pose->liftDmm);
    serialSendString(" t=");
    serialSendUint(pose->turretDdeg);
    serialSendString("\r\n");
}

static void startMechanismPose(const MechanismPose *target)
{
    int16_t horizontalDelta = (int16_t)(target->horizontalDmm -
                                        mechanismState.current.horizontalDmm);
    int16_t liftDelta = (int16_t)((int32_t)target->liftDmm -
                                  (int32_t)mechanismState.current.liftDmm);
    uint32_t horizontalPulses = mechanismHorizontalPulses(horizontalDelta);
    uint32_t liftPulses = mechanismLiftPulses(liftDelta);

    if (!mechanismStateStart(&mechanismState, target, clockMs)) {
        serialSendString("ERR MECH: invalid or busy pose\r\n");
        return;
    }
    setAuxMotorsEnabled(true);
    if (motionInterrupted()) { stopAllMotors(); return; }
    if (horizontalPulses != 0U) {
        Emm_V5_Pos_Control(6U, horizontalDelta > 0 ? 0U : 1U,
                           target->horizontalRpm, target->horizontalAccel,
                           horizontalPulses, false, true);
    }
    if (motionInterrupted()) { stopAllMotors(); return; }
    if (liftPulses != 0U) {
        Emm_V5_Pos_Control(5U, liftDelta > 0 ? 0U : 1U,
                           target->liftRpm, target->liftAccel,
                           liftPulses, false, true);
    }
    servoSetTargetMdeg(4U, (uint32_t)target->turretDdeg * 100U,
                       target->turretDps10);
    if (motionInterrupted()) { stopAllMotors(); return; }
    Emm_V5_Synchronous_motion(0x00);
    printMechanismPose("MECH RUN", target);
}

static uint8_t processMechanismCommand(const char *commandText)
{
    MechanismCommand command;
    if (strncmp(commandText, "mech ", 5U) != 0) return 0U;
    if (!mechanismParseCommand(commandText, &command)) {
        serialSendString("ERR MECH: use init H L T, pose H L T HR HA LR LA TS, or status\r\n");
        return 1U;
    }
    if (command.type == MECHANISM_COMMAND_STATUS) {
        if (!mechanismState.valid) {
            serialSendString("MECH INVALID reason=not_initialized\r\n");
        } else {
            printMechanismPose(mechanismState.running ? "MECH RUN" : "MECH POS",
                               mechanismState.running ? &mechanismState.target :
                                                        &mechanismState.current);
        }
        return 1U;
    }
    if (motionMode || routeActive || mechanismState.running ||
        mechanismActionActive) {
        serialSendString("ERR MECH: busy; stop first\r\n");
        return 1U;
    }
    if (command.type == MECHANISM_COMMAND_INIT) {
        mechanismStateInitialize(&mechanismState, &command.pose);
        servoSetPulse(4U, servoAngleToPulse(
            4U, (uint32_t)command.pose.turretDdeg * 100U));
        currentAngleMdeg[2] = (uint32_t)command.pose.turretDdeg * 100U;
        targetAngleMdeg[2] = currentAngleMdeg[2];
        servoChannelMoving[2] = 0U;
        printMechanismPose("MECH INIT", &command.pose);
        return 1U;
    }
    if (command.type != MECHANISM_COMMAND_POSE) {
        serialSendString("ERR MECH: unsupported command\r\n");
        return 1U;
    }
    if (!mechanismState.valid) {
        serialSendString("ERR MECH: initialize current pose first\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return 1U;
    }
    armed = 0U;
    startMechanismPose(&command.pose);
    return 1U;
}

static void serviceMechanism(void)
{
    uint8_t event = mechanismStateService(&mechanismState, clockMs);
    if (event == MECHANISM_EVENT_POSITION) {
        printMechanismPose("MECH POS", &mechanismState.current);
    } else if (event == MECHANISM_EVENT_DONE) {
        stopAuxMotors();
        printMechanismPose("MECH DONE", &mechanismState.current);
    }
}

uint8_t mechanismActionStart(const MechanismInitialState *initial,
                             const MechanismAction *actions,
                             uint16_t count)
{
    if (initial == (const MechanismInitialState *)0 ||
        actions == (const MechanismAction *)0 || count == 0U ||
        mechanismActionActive || !mechanismState.valid ||
        mechanismState.running || motionMode ||
        (routeActive && !(routeWaiting &&
                         ((rawPickRouteActive && rawPickAtStation) || missionActive))))
        return 0U;
    mechanismActionInitial = initial;
    mechanismActions = actions;
    mechanismActionCount = count;
    mechanismActionIndex = 0U;
    mechanismActionDispatched = 0U;
    mechanismActionActive = 1U;
    mechanismActionWaitUntil = 0U;
    return 1U;
}

static void mechanismActionCompleteStep(uint16_t waitMs)
{
    if (waitMs != 0U) {
        mechanismActionWaitUntil = clockMs + waitMs;
        return;
    }
    ++mechanismActionIndex;
    mechanismActionDispatched = 0U;
}

void mechanismActionService(void)
{
    const MechanismAction *action;
    uint16_t angle;
    uint16_t speedDps10;
    if (!mechanismActionActive) return;
    if (mechanismActionDispatched && mechanismActionWaitUntil != 0U) {
        if ((int32_t)(clockMs - mechanismActionWaitUntil) < 0) return;
        mechanismActionWaitUntil = 0U;
        ++mechanismActionIndex;
        mechanismActionDispatched = 0U;
    }
    if (mechanismActionIndex >= mechanismActionCount) {
        mechanismActionActive = 0U;
        return;
    }
    action = &mechanismActions[mechanismActionIndex];
    if (!mechanismActionDispatched) {
        if (action->type == MECHANISM_ACTION_POSE) {
            startMechanismPose(&action->pose);
        } else if (action->type == MECHANISM_ACTION_GRIPPER) {
            angle = action->value ? mechanismActionInitial->gripperOpenDeg :
                                    mechanismActionInitial->gripperCloseDeg;
            servoSetTarget(2U, angle, mechanismActionInitial->gripperDps10);
        } else if (action->type == MECHANISM_ACTION_PLATFORM) {
            if (action->value < 1U || action->value > 3U) {
                mechanismActionActive = 0U;
                return;
            }
            servoSetTarget(3U,
                mechanismActionInitial->platformDeg[action->value - 1U],
                mechanismActionInitial->platformDps10);
        } else if (action->type == MECHANISM_ACTION_SERVO) {
            if (action->channel < 2U || action->channel > 4U ||
                (action->channel < 4U && action->value > 270U) ||
                (action->channel == 4U && action->value > 360U)) {
                mechanismActionActive = 0U;
                return;
            }
            speedDps10 = action->channel == 2U ?
                mechanismActionInitial->gripperDps10 :
                (action->channel == 3U ?
                    mechanismActionInitial->platformDps10 :
                    mechanismActionInitial->pose.turretDps10);
            servoSetTarget(action->channel, action->value, speedDps10);
        } else if (action->type != MECHANISM_ACTION_WAIT) {
            mechanismActionActive = 0U;
            return;
        }
        mechanismActionDispatched = 1U;
        if (!mechanismActionActive) return;
        if (action->type == MECHANISM_ACTION_WAIT) {
            mechanismActionWaitUntil = clockMs + action->waitMs;
            return;
        }
    }
    if (action->type == MECHANISM_ACTION_POSE && mechanismState.running) return;
    if ((action->type == MECHANISM_ACTION_GRIPPER ||
         action->type == MECHANISM_ACTION_PLATFORM ||
         action->type == MECHANISM_ACTION_SERVO) &&
        servoChannelMoving[action->channel - 2U]) return;
    mechanismActionCompleteStep(action->waitMs);
}

static void startLine(const char *cursor, uint8_t heading)
{
    uint16_t amount, rpm;
    uint8_t id, direction;
    char way = *cursor++;
    uint32_t pulses;
    if ((heading == 2U ? (way != 'A' && way != 'D') :
         (way != 'W' && way != 'S' && (!heading || (way != 'A' && way != 'D')))) ||
        *cursor != ' ' ||
        !parsePair(cursor, &amount, &rpm) ||
        !motionRequestValid(heading, amount, rpm) ||
        (heading == 2U && rpm > routeLateralRpmLimit)) {
        serialSendString("ERR: line W|S 100..500(step100); straight W|S|A|D 1000..5000 rpm 10..120; pid move A|D 1000..5000 rpm 10..lateral limit\r\n");
        return;
    }
    if (!armed) { serialSendString("ERR: send 'arm' first\r\n"); return; }
    armed = 0U;
    invalidateNavigation("manual_motion");
    if (heading && (!imuValid || clockMs - imuStamp > 250U)) {
        serialSendString("ERR: fresh IMU yaw required; check baud/wiring/status\r\n");
        return;
    }
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {
        stopAllMotors();
        return;
    }
    if (heading) {
        targetYaw = imuYaw; /* Hold the current heading; no sensor zero command. */
        straightRpm = rpm;
        straightDirection = (way == 'W' || way == 'D') ? 1 : -1;
        straightLateral = (uint8_t)(way == 'A' || way == 'D');
        straightUseRoutePid = (uint8_t)(heading == 2U);
        if (straightUseRoutePid) {
            routeHeadingPidReset(&routeHeadingPid);
            headingPidLastReport = clockMs - 200U;
        }
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
    serialSendString(heading == 2U ? "RUN PID lateral heading hold; auto-disarmed\r\n" :
                     heading ? "RUN heading hold; auto-disarmed\r\n" :
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
    invalidateNavigation("manual_motion");
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {
        stopAllMotors();
        return;
    }

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

static void startAuxMotorJog(uint8_t motorId, uint8_t direction)
{
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return;
    }
    armed = 0U;

    Emm_V5_En_Control(motorId, true, false);
    delay_ms(2U);
    if (motionInterrupted()) {
        stopAllMotors();
        return;
    }
    Emm_V5_Pos_Control(motorId, direction,
                       MOTOR_TEST_SPEED, MOTOR_TEST_ACCEL,
                       MOTOR_TEST_PULSES, false, false);
    serialSendString("TX queued: motor ");
    serialSendUint(motorId);
    serialSendString(" direction ");
    serialSendString(direction == 0U ? "0" : "1");
    serialSendString(", auto-disarmed\r\n");
}

static uint8_t processAuxMoveCommand(const char *command)
{
    const char *cursor;
    uint16_t motorId, rpm, accel;
    int16_t distanceDmm;
    int16_t maximumDmm;
    uint32_t pulses;
    uint8_t direction;

    if (strncmp(command, "auxmove ", 8U) != 0) return 0U;
    cursor = command + 8U;
    if (!mechanismParseUnsigned(&cursor, &motorId) ||
        !mechanismParseSigned(&cursor, &distanceDmm) ||
        !mechanismParseUnsigned(&cursor, &rpm) ||
        !mechanismParseUnsigned(&cursor, &accel) ||
        !mechanismAtEnd(cursor)) {
        serialSendString("ERR: auxmove 5|6 signed_dmm rpm accel\r\n");
        return 1U;
    }
    if (motorId < AUX_MOTOR_MIN_ID || motorId > AUX_MOTOR_MAX_ID) {
        serialSendString("ERR: auxmove motor 5|6\r\n");
        return 1U;
    }
    maximumDmm = motorId == 5U ? 1500 : 1870;
    if (distanceDmm == 0 || distanceDmm < -maximumDmm ||
        distanceDmm > maximumDmm || rpm < MECHANISM_RPM_MIN ||
        rpm > MECHANISM_RPM_MAX || accel < MECHANISM_ACCEL_MIN ||
        accel > MECHANISM_ACCEL_MAX) {
        serialSendString("ERR: auxmove distance/rpm/accel out of range\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return 1U;
    }
    armed = 0U;
    invalidateMechanism("manual_auxmove");
    pulses = motorId == 5U ? mechanismLiftPulses(distanceDmm) :
                             mechanismHorizontalPulses(distanceDmm);
    direction = distanceDmm > 0 ? 0U : 1U;
    Emm_V5_En_Control((uint8_t)motorId, true, false);
    delay_ms(2U);
    if (motionInterrupted()) {
        stopAllMotors();
        return 1U;
    }
    Emm_V5_Pos_Control((uint8_t)motorId, direction, rpm,
                       (uint8_t)accel, pulses, false, false);
    motionMode = 3U;
    auxMoveMotorId = (uint8_t)motorId;
    motionStart = clockMs;
    lastControl = clockMs;
    motionDuration = mechanismMotorDurationMs(pulses, rpm) * 2U +
                     AUX_MOVE_TIMEOUT_MARGIN_MS;
    serialSendString("TX queued: auxmove motor=");
    serialSendUint(motorId);
    serialSendString(" distance_dmm=");
    serialSendInt(distanceDmm);
    serialSendString(", auto-disarmed\r\n");
    return 1U;
}

static void checkMotorCan(uint8_t motorId)
{
    JogCanReply reply;
    uint8_t result;
    uint8_t i;
    result = jogCanReadStatus(motorId, &emergencyStop, &reply);
    if (result == 0U) return;

    if (result == 1U) {
        serialSendString("CAN RX motor ");
        serialSendUint(motorId);
        serialSendString(": ExtId=0x");
        serialSendHex32(reply.extId);
        serialSendString(" DLC=0x");
        serialSendHex8(reply.dlc);
        serialSendString(" DATA=");
        for (i = 0U; i < reply.dlc; ++i) {
            serialSendHex8(reply.data[i]);
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
    serialSendString(" forward_scale_bp=");
    serialSendUint((uint16_t)(routeForwardScale * 10000.0f + 0.5f));
    serialSendString(" lateral_scale_bp=");
    serialSendUint((uint16_t)(routeLateralScale * 10000.0f + 0.5f));
    serialSendString(" turn_rpm=");
    serialSendUint(routeTurnRpmLimit);
    serialSendString(" lateral_rpm_limit=");
    serialSendUint(routeLateralRpmLimit);
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

static void printHeadingPidStatus(void)
{
    serialSendString("PID kp_x100=");
    serialSendUint((uint16_t)(routeHeadingGains.kp * 100.0f + 0.5f));
    serialSendString(" ki_x100=");
    serialSendUint((uint16_t)(routeHeadingGains.ki * 100.0f + 0.5f));
    serialSendString(" kd_x100=");
    serialSendUint((uint16_t)(routeHeadingGains.kd * 100.0f + 0.5f));
    serialSendString(" RAM_only\r\n");
}

static void printHeadingPidTrace(float targetYaw, float actualYaw, int16_t turn)
{
    serialSendString("PID TRACE target_cdeg=");
    serialSendInt((int16_t)(headingError(targetYaw, 0.0f) * 100.0f));
    serialSendString(" actual_cdeg=");
    serialSendInt((int16_t)(actualYaw * 100.0f));
    serialSendString(" output_rpm=");
    serialSendInt(turn);
    serialSendString("\r\n");
}

static int16_t navFieldYawCdeg(void)
{
    float fieldYaw = 90.0f + headingError(imuYaw, routeBaseYaw) * yawSign;
    while (fieldYaw > 180.0f) fieldYaw -= 360.0f;
    while (fieldYaw <= -180.0f) fieldYaw += 360.0f;
    return (int16_t)(fieldYaw * 100.0f);
}

static void printNavPose(const char *state)
{
    NavPoint target = navPathCount > 0U ? navPath[navPathCount - 1U]
                                        : navPoint(navCurrentNode);
    serialSendString("NAV POS x=");
    serialSendUint((uint16_t)(routeX + 0.5f));
    serialSendString(" y=");
    serialSendUint((uint16_t)(routeY + 0.5f));
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString(" state=");
    serialSendString(state);
    serialSendString(" target_x=");
    serialSendUint((uint16_t)target.x);
    serialSendString(" target_y=");
    serialSendUint((uint16_t)target.y);
    serialSendString("\r\n");
}

static RoutePoint activeRoutePoint(void)
{
    if (navRunning) {
        NavPoint point = navPath[routeIndex];
        RoutePoint result;
        result.x = point.x;
        result.y = point.y;
        result.event = 0;
        result.heading = routeIndex + 1U == navPathCount ?
                         point.arrivalHeading : NAV_HEADING_KEEP;
        return result;
    }
    return routePoint(routeIndex, routeStartZone);
}

static const char *routeStageName(const RoutePoint *point)
{
    return point->event != (const char *)0 ? point->event : "TRANSIT";
}

static void printFullRoutePose(const char *state)
{
    RoutePoint target = activeRoutePoint();
    serialSendString("ROUTE POS x=");
    serialSendUint((uint16_t)(routeX + 0.5f));
    serialSendString(" y=");
    serialSendUint((uint16_t)(routeY + 0.5f));
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString(" state=");
    serialSendString(state);
    serialSendString(" stage=");
    serialSendString(routeStageName(&target));
    serialSendString(" target_x=");
    serialSendUint((uint16_t)target.x);
    serialSendString(" target_y=");
    serialSendUint((uint16_t)target.y);
    serialSendString("\r\n");
}

static void printFullRouteStage(const RoutePoint *point)
{
    serialSendString("ROUTE STAGE index=");
    serialSendUint((uint16_t)(routeIndex + 1U));
    serialSendString(" name=");
    serialSendString(routeStageName(point));
    serialSendString("\r\n");
}

static void finishFullRoute(void)
{
    int16_t x = (int16_t)(routeX + 0.5f);
    int16_t y = (int16_t)(routeY + 0.5f);
    stopAllMotors();
    serialSendString("ROUTE DONE x=");
    serialSendUint((uint16_t)x);
    serialSendString(" y=");
    serialSendUint((uint16_t)y);
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString("\r\n");
    serialSendString("ROUTE END: verify actual home position\r\n");
}

static void finishNavigation(void)
{
    NavPoint target = navPath[navPathCount - 1U];
    stopDriveMotors();
    routeActive = 0U;
    routeWaiting = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeHeadingPidReset(&routeHeadingPid);
    routeForward = routeRight = 0;
    navRunning = 0U;
    navCurrentNode = navNodeAt(target.x, target.y);
    routeX = target.x;
    routeY = target.y;
    printNavPose("IDLE");
    serialSendString("NAV DONE x=");
    serialSendUint((uint16_t)target.x);
    serialSendString(" y=");
    serialSendUint((uint16_t)target.y);
    serialSendString(" yaw_cdeg=");
    serialSendInt(navFieldYawCdeg());
    serialSendString("\r\n");
}

static void serviceRoute(void)
{
    uint32_t now, dt, age, sampleMs;
    RoutePoint p;
    float yaw, error, dx, dy, remaining;
    int16_t speed, turn, bodyForward, bodyRight;
    float pidTurn;
    uint16_t cruiseRpm;
    uint8_t lateral;
    if (!routeActive || motionInterrupted()) return;
    now = clockMs;
    if (routeWaiting) { routeTick = now; return; }
    dt = now - routeTick;
    if (dt < 20U) return;
    __disable_irq();
    yaw = imuYaw; sampleMs = imuStamp; age = clockMs - sampleMs;
    __enable_irq();
    error = headingError(routeYaw, yaw);
    if (!imuValid || age > 250U || (!routeRotating && routeAbs(error) > 20.0f) || dt > 150U ||
        now - routeLegStart > 45000U) {
        uint8_t wasNavigation = navRunning;
        uint8_t wasFullRoute = fullRouteRunning;
        stopAllMotors();
        if (wasNavigation) serialSendString("NAV INVALID reason=route_fault\r\n");
        if (wasFullRoute) serialSendString("ROUTE INVALID reason=route_fault\r\n");
        serialSendString(wasNavigation ?
            "ERR NAV: IMU/deviation/control delay/leg timeout; position invalid\r\n" :
            "ERR ROUTE: IMU/deviation/control delay/leg timeout; aborted\r\n");
        return;
    }
    routeTick = now;
    if (routeRotating) {
        if (navRunning && now - navLastReport >= 200U) {
            navLastReport = now;
            printNavPose("TURN");
        }
        if (fullRouteRunning && now - routeLastReport >= 200U) {
            routeLastReport = now;
            printFullRoutePose("TURN");
        }
        if (now - routeTurnStart > 18000U) {
            uint8_t wasNavigation = navRunning;
            uint8_t wasFullRoute = fullRouteRunning;
            stopAllMotors();
            if (wasNavigation) serialSendString("NAV INVALID reason=turn_timeout\r\n");
            if (wasFullRoute) serialSendString("ROUTE INVALID reason=turn_timeout\r\n");
            serialSendString(wasNavigation ?
                "ERR NAV: turn timeout; position invalid\r\n" :
                "ERR TURN: timeout; aborted\r\n");
            return;
        }
        /* Separate enter/exit thresholds prevent stop/start chatter at 2deg. */
        if (!routeTurnInBand && routeAbs(error) <= 2.0f) routeTurnInBand = 1U;
        if (routeTurnInBand && routeAbs(error) > 3.5f) routeTurnInBand = 0U;
        routeTurnRpm = routeSlew(routeTurnRpm,
            routeTurnInBand ? 0.0f :
            (float)routeTurnSpeedLimited(error, yawSign, routeTurnRpmLimit),
            ROUTE_TURN_ACCEL, dt);
        sendRouteSpeeds(0, 0, routeRound(routeTurnRpm));
        if (routeTurnInBand && routeRound(routeTurnRpm) == 0) {
            if (routeTurnInBand == 1U) { routeTurnStable = now; routeTurnInBand = 2U; }
            if (now - routeTurnStable >= 200U) {
                routeRotating = routeTurnInBand = 0U;
                routeHeading = routeNextHeading;
                routeLegStart = now;
                routeHeadingPidReset(&routeHeadingPid);
                if (routeOnlyTurn) {
                    stopAllMotors(); serialSendString("TURN DONE (2deg entry / 3.5deg hysteresis)\r\n");
                }
            }
        }
        return;
    }
    /* Integration only estimates travel. IMU constrains yaw, not XY drift. */
    routeX += routeEstimateScaled(routeRight, dt,
        (routeHeading == 0 || routeHeading == 2) ?
        routeLateralScale : routeForwardScale);
    routeY += routeEstimateScaled(routeForward, dt,
        (routeHeading == 1 || routeHeading == -1) ?
        routeLateralScale : routeForwardScale);
    if (navRunning && now - navLastReport >= 200U) {
        navLastReport = now;
        printNavPose("RUN");
    }
    if (fullRouteRunning && now - routeLastReport >= 200U) {
        routeLastReport = now;
        printFullRoutePose("RUN");
    }
    if (!routeAuto && now - routeSettle < 300U) return;
    p = activeRoutePoint();
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
        routeHeadingPidReset(&routeHeadingPid);
        routeX = p.x; routeY = p.y; /* snap NOMINAL coordinates only */
        if (p.heading != 4 && p.heading != routeHeading) {
            routeNextHeading = p.heading;
            routeYaw = routeBaseYaw + p.heading * 90.0f * yawSign;
            routeRotating = 1U; routeTurnInBand = 0U;
            routeTurnStart = routeLegStart = clockMs;
            serialSendString("ROUTE turning to station heading\r\n");
            return;
        }
        if (navRunning) {
            navCurrentNode = navNodeAt(navPath[routeIndex].x,
                                       navPath[routeIndex].y);
            if (routeIndex + 1U == navPathCount) {
                finishNavigation();
                return;
            }
            ++routeIndex;
            routeSettle = routeLegStart = clockMs;
            return;
        }
        serialSendString("ROUTE estimated waypoint: ");
        serialSendUint((uint16_t)(routeIndex + 1U));
        if (p.event) { serialSendChar(' '); serialSendString(p.event); }
        serialSendString("\r\n");
        if (fullRouteRunning && p.event) printFullRouteStage(&p);
        if (routeIndex + 1U == ROUTE_COUNT) {
            if (fullRouteRunning) finishFullRoute();
            else {
                stopAllMotors();
                serialSendString("ROUTE END: verify actual home position\r\n");
            }
            return;
        }
        ++routeIndex;
        routeWaiting = (uint8_t)((!routeAuto && (routeStep || p.event != 0)) ||
                                 (rawPickRouteActive && routeIndex == 4U) ||
                                 (missionActive && p.event != 0));
        routeSettle = routeLegStart = clockMs;
        if (missionActive) missionBeginStation(routeIndex);
        if (rawPickRouteActive && routeIndex == 4U) {
            rawPickAtStation = 1U;
            rawPickItemIndex = 0U;
            rawPickMissingReported = 0U;
            serialSendString("ROUTE RAWPICK state=WAIT_MATERIAL color=4 slot=1\r\n");
        }
        if (routeWaiting && !missionActive)
            serialSendString("WAIT: verify position / finish station work; send route next\r\n");
        return;
    }
    /* Map-axis translation is transformed to the current body heading. */
    lateral = routeAbs(dx) > 1.5f;
    remaining = lateral ? routeAbs(dx) : routeAbs(dy);
    cruiseRpm = lateral && routeRpm > routeLateralRpmLimit ?
        routeLateralRpmLimit : routeRpm;
    speed = routeSpeed(remaining, now - routeLegStart, cruiseRpm);
    routeDriveRpm = routeSlew(routeDriveRpm, speed, ROUTE_ACCEL_RPM_S, dt);
    speed = routeRound(routeDriveRpm);
    routeRight = lateral ? (dx > 0 ? speed : -speed) : 0;
    routeForward = lateral ? 0 : (dy > 0 ? speed : -speed);
    pidTurn = routeHeadingPidStep(&routeHeadingPid, &routeHeadingGains, error,
                                  yaw, sampleMs, dt, yawSign);
    routeCorrection = routeSlew(routeCorrection,
        pidTurn * speed / cruiseRpm, 90.0f, dt);
    turn = routeRound(routeCorrection);
    routeBody(routeForward, routeRight, routeHeading, &bodyForward, &bodyRight);
    sendRouteSpeeds(bodyForward, bodyRight, turn);
    if (!motionInterrupted() && now - headingPidLastReport >= 200U) {
        headingPidLastReport = now;
        printHeadingPidTrace(routeYaw, yaw, turn);
    }
}

static uint8_t processNavCommand(const char *command)
{
    NavPoint startPoint, targetPoint;
    uint16_t x;
    uint16_t y;
    uint16_t rpm;
    uint8_t node;

    if (strcmp(command, "nav status") == 0) {
        if (!navInitialized) serialSendString("NAV INVALID\r\n");
        else printNavPose(navRunning ? (routeRotating ? "TURN" : "RUN") : "IDLE");
        return 1U;
    }
    if (strcmp(command, "nav init 1") == 0 || strcmp(command, "nav init 2") == 0) {
        if (motionMode || routeActive) {
            serialSendString("ERR NAV: init requires idle; stop first\r\n");
            return 1U;
        }
        if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
            serialSendString("ERR NAV: fresh IMU and healthy CAN required\r\n");
            return 1U;
        }
        node = navStartNode((uint8_t)(command[9] - '0'));
        startPoint = navPoint(node);
        armed = 0U;
        navInitialized = 1U;
        navRunning = 0U;
        navCurrentNode = node;
        navPathCount = 0U;
        routeX = startPoint.x;
        routeY = startPoint.y;
        routeHeading = routeNextHeading = NAV_HEADING_UP;
        routeBaseYaw = routeYaw = imuYaw;
        serialSendString("NAV INIT x=");
        serialSendUint((uint16_t)startPoint.x);
        serialSendString(" y=");
        serialSendUint((uint16_t)startPoint.y);
        serialSendString(" yaw_cdeg=9000\r\n");
        return 1U;
    }
    if (strncmp(command, "nav goto ", 9U) != 0) return 0U;
    if (!parsePairWithOptionalRpm(command + 9, &x, &y, &rpm) ||
        rpm < 10U || rpm > 120U) {
        serialSendString("ERR NAV: use nav goto X Y [10..120rpm]\r\n");
        return 1U;
    }
    if (x > 2400U || y > 2400U ||
        !navPointClear((int16_t)x, (int16_t)y)) {
        serialSendString("ERR NAV: target has no 300mm body clearance\r\n");
        return 1U;
    }
    if (!navInitialized) {
        serialSendString("ERR NAV: select nav init 1|2 first\r\n");
        return 1U;
    }
    if (motionMode || routeActive) {
        serialSendString("ERR NAV: busy; stop first\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString("ERR: send 'arm' first\r\n");
        return 1U;
    }
    armed = 0U;
    if (!imuValid || clockMs - imuStamp > 250U ||
        routeAbs(headingError(routeYaw, imuYaw)) > 20.0f || motionInterrupted()) {
        invalidateNavigation("imu_or_heading_fault");
        serialSendString("ERR NAV: fresh IMU/original heading required; position invalid\r\n");
        return 1U;
    }
    startPoint.x = (int16_t)(routeX + 0.5f);
    startPoint.y = (int16_t)(routeY + 0.5f);
    startPoint.arrivalHeading = NAV_HEADING_KEEP;
    targetPoint.x = (int16_t)x;
    targetPoint.y = (int16_t)y;
    targetPoint.arrivalHeading = NAV_HEADING_KEEP;
    navPathCount = navPlanPath(startPoint, targetPoint, navPath,
                               NAV_PATH_CAPACITY);
    if (navPathCount == 0U) {
        invalidateNavigation("no_safe_path");
        serialSendString("ERR NAV: no safe path; position invalid\r\n");
        return 1U;
    }
    navRunning = 1U;
    fullRouteRunning = 0U;
    routeRpm = rpm;
    routeActive = routeAuto = 1U;
    routeWaiting = routeStep = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeIndex = 0U;
    routeForward = routeRight = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = 0U;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {
        stopAllMotors();
        serialSendString("NAV INVALID reason=can_fault\r\n");
        serialSendString("ERR NAV: CAN failure; position invalid\r\n");
        return 1U;
    }
    routeTick = routeSettle = routeLegStart = clockMs;
    navLastReport = clockMs - 200U;
    printNavPose("RUN");
    return 1U;
}

static uint8_t processRouteCommand(const char *command)
{
    uint8_t start, step, autoRun, rawRun = 0U, missionRun = 0U;
    uint16_t degrees, rpm, forwardBp, lateralBp, lateralRpm, kp100, ki100, kd100;
    const char *cursor;
    if (strcmp(command, "pid get") == 0) {
        printHeadingPidStatus();
        return 1U;
    }
    if (strncmp(command, "pid set ", 8U) == 0) {
        cursor = command + 8U;
        if (!parseUint(&cursor, &kp100) || !parseUint(&cursor, &ki100) ||
            !parseUint(&cursor, &kd100) || *cursor != '\0' ||
            kp100 > 1000U || ki100 > 500U || kd100 > 500U) {
            serialSendString("ERR PID: set KP KI KD (x100); KP 0..1000, KI/KD 0..500\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running ||
            mechanismActionActive) {
            serialSendString("ERR PID: stop motion before tuning\r\n");
            return 1U;
        }
        routeHeadingGains.kp = kp100 / 100.0f;
        routeHeadingGains.ki = ki100 / 100.0f;
        routeHeadingGains.kd = kd100 / 100.0f;
        routeHeadingPidReset(&routeHeadingPid);
        serialSendString("OK PID RAM_only\r\n");
        printHeadingPidStatus();
        return 1U;
    }
    if (strcmp(command, "route status") == 0) { printRouteStatus(); return 1U; }
    if (strncmp(command, "route tune ", 11U) == 0) {
        cursor = command + 11U;
        if (!parseUint(&cursor, &forwardBp) ||
            !parseUint(&cursor, &lateralBp) ||
            !parseUint(&cursor, &rpm)) {
            serialSendString("ERR ROUTE: tune F L T [V]; scales 5000..15000, speeds 10..120rpm\r\n");
            return 1U;
        }
        lateralRpm = routeLateralRpmLimit;
        if (*cursor == ' ' && !parseUint(&cursor, &lateralRpm)) {
            serialSendString("ERR ROUTE: tune F L T [V]; scales 5000..15000, speeds 10..120rpm\r\n");
            return 1U;
        }
        if (*cursor != '\0' ||
            forwardBp < 5000U || forwardBp > 15000U ||
            lateralBp < 5000U || lateralBp > 15000U ||
            rpm < 10U || rpm > 120U ||
            lateralRpm < 10U || lateralRpm > 120U) {
            serialSendString("ERR ROUTE: tune F L T [V]; scales 5000..15000, speeds 10..120rpm\r\n");
            return 1U;
        }
        if (motionMode || routeActive) {
            serialSendString("ERR ROUTE: tune requires idle; stop first\r\n");
            return 1U;
        }
        routeForwardScale = forwardBp / 10000.0f;
        routeLateralScale = lateralBp / 10000.0f;
        routeTurnRpmLimit = rpm;
        routeLateralRpmLimit = lateralRpm;
        serialSendString("OK route tune RAM_only\r\n");
        printRouteStatus();
        return 1U;
    }
    if (strncmp(command, "route scale ", 12U) == 0) {
        cursor = command + 12U;
        if (!parseUint(&cursor, &degrees) || *cursor != '\0' ||
            degrees < 5000U || degrees > 15000U) {
            serialSendString("ERR ROUTE: scale must be 5000..15000 (50.00%..150.00%)\r\n");
            return 1U;
        }
        if (motionMode || routeActive) {
            serialSendString("ERR ROUTE: scale requires idle; stop first\r\n");
            return 1U;
        }
        routeLateralScale = degrees / 10000.0f;
        serialSendString("OK route lateral_scale_bp=");
        serialSendUint(degrees);
        serialSendString(" RAM_only\r\n");
        return 1U;
    }
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
        routeHeadingPidReset(&routeHeadingPid);
        routeSentValid = 0U;
        routeTick = routeTurnStart = routeLegStart = clockMs;
        routeWaiting = routeTurnInBand = 0U;
        routeOnlyTurn = routeRotating = routeActive = 1U;
        serialSendString("TURN running\r\n"); return 1U;
    }
    if (strcmp(command, "route next") == 0) {
        if (missionActive) {
            serialSendString("ERR ROUTE: mission advances automatically\r\n");
            return 1U;
        }
        if (!routeActive || !routeWaiting) {
            serialSendString("ERR: no waiting route\r\n"); return 1U;
        }
        if (rawPickAtStation && rawPickItemIndex < 3U) {
            serialSendString("ERR ROUTE: finish three raw pickups before route next\r\n");
            return 1U;
        }
        if (!imuValid || clockMs - imuStamp > 250U ||
            routeAbs(headingError(routeYaw, imuYaw)) > 20.0f) {
            serialSendString("ERR: restore fresh IMU / original heading first\r\n"); return 1U;
        }
        routeWaiting = 0U;
        rawPickAtStation = rawPickRouteActive = 0U;
        routeTick = routeSettle = routeLegStart = clockMs;
        serialSendString("ROUTE continuing\r\n"); return 1U;
    }
    if (strncmp(command, "route start ", 12U) == 0) {
        cursor = command + 12U;
        step = autoRun = 0U;
    } else if (strncmp(command, "route step ", 11U) == 0) {
        cursor = command + 11U;
        step = 1U; autoRun = 0U;
    } else if (strncmp(command, "route auto ", 11U) == 0) {
        cursor = command + 11U;
        step = 0U; autoRun = 1U;
    } else if (strncmp(command, "route rawpick ", 14U) == 0) {
        cursor = command + 14U;
        step = 0U; autoRun = rawRun = 1U;
    } else if (strncmp(command, "route mission ", 14U) == 0) {
        cursor = command + 14U;
        step = 0U; autoRun = missionRun = 1U;
    } else return 0U;
    if (!parseValueWithOptionalRpm(cursor, &degrees, &rpm) ||
        degrees < 1U || degrees > 2U || rpm < 10U || rpm > 120U) {
        serialSendString("ERR: route start|step|auto|rawpick|mission 1|2 [10..120rpm]\r\n");
        return 1U;
    }
    if (motionMode || routeActive || mechanismState.running ||
        mechanismActionActive || visionSessionActive()) {
        serialSendString(missionRun ? "ERR ROUTE: mission busy; stop first\r\n" :
                                      "ERR: busy; stop first\r\n");
        return 1U;
    }
    if (!armed) {
        serialSendString(missionRun ? "ERR ROUTE: mission needs arm\r\n" :
                                      "ERR: send 'arm' first\r\n");
        return 1U;
    }
    if ((rawRun || missionRun) && !mechanismState.valid) {
        serialSendString("ERR ROUTE: rawpick mechanism not initialized; check mech status\r\n");
        return 1U;
    }
    if ((rawRun || missionRun) && (mechanismState.current.horizontalDmm != 0 ||
                   mechanismState.current.liftDmm != 0U ||
                   mechanismState.current.turretDdeg != 2700U)) {
        serialSendString("ERR ROUTE: rawpick needs pose h=0 l=0 t=2700; check mech status\r\n");
        return 1U;
    }
    if ((rawRun || missionRun) && (servoChannelMoving[0] || servoChannelMoving[1] ||
                   servoChannelMoving[2])) {
        serialSendString("ERR ROUTE: rawpick servo still moving; wait for SERVO DONE\r\n");
        return 1U;
    }
    if ((rawRun || missionRun) && !vision_calibration_valid(&visionMaterialCalibration)) {
        serialSendString("ERR ROUTE: rawpick material calibration invalid; apply vision scale material\r\n");
        return 1U;
    }
    if (missionRun && !vision_calibration_valid(&visionRingCalibration[1])) {
        serialSendString("ERR ROUTE: mission ring 2 calibration invalid; apply vision scale ring 2\r\n");
        return 1U;
    }
    if (missionRun && (!servoChannelEnabled[0] || !servoChannelEnabled[1] ||
                       currentAngleMdeg[0] != 70000U ||
                       currentAngleMdeg[1] != 26000U)) {
        serialSendString("ERR ROUTE: mission needs commanded gripper=70 and platform=26\r\n");
        return 1U;
    }
    armed = 0U;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        serialSendString(missionRun ?
            "ERR ROUTE: mission needs fresh IMU and healthy CAN\r\n" :
            "ERR: fresh IMU and healthy CAN required\r\n");
        return 1U;
    }
    invalidateNavigation("manual_motion");
    start = (uint8_t)degrees;
    routeRpm = rpm;
    routeAuto = autoRun; fullRouteRunning = autoRun;
    missionActive = missionRun;
    missionPhase = MISSION_IDLE;
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    rawPickRouteActive = rawRun;
    rawPickAtStation = rawPickItemIndex = rawPickMissingReported = 0U;
    routeHeading = routeNextHeading = 0;
    routeDriveRpm = routeTurnRpm = routeCorrection = 0.0f;
    routeHeadingPidReset(&routeHeadingPid);
    routeSentValid = 0U;
    routeRotating = routeOnlyTurn = routeTurnInBand = 0U;
    routeStartZone = start; routeStep = step; routeIndex = 0U;
    routeX = 2250.0f; routeY = start == 1U ? 2250.0f : 150.0f;
    routeBaseYaw = routeYaw = imuYaw;
    routeForward = routeRight = 0;
    setAllMotorsEnabled(true);
    if (motionInterrupted()) {
        uint8_t wasFullRoute = fullRouteRunning;
        stopAllMotors();
        if (wasFullRoute) serialSendString("ROUTE INVALID reason=can_fault\r\n");
        return 1U;
    }
    routeTick = routeSettle = routeLegStart = clockMs;
    routeLastReport = clockMs - 200U;
    routeWaiting = 0U; routeActive = 1U;
    if (rawRun || missionRun) vision_session_init(&visionSession);
    serialSendString("ROUTE START: nose UP, clear floor, nominal distance only\r\n");
    if (fullRouteRunning) printFullRoutePose("RUN");
    return 1U;
}

static void serviceRawPickRoute(void)
{
    uint8_t color;
    if (!rawPickAtStation || !routeWaiting) return;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        visionPickActive = 1U;
        failVision("IMU_OR_CAN_FAULT");
        return;
    }
    if (rawPickItemIndex >= 3U) return;
    if (visionPickActive || visionSessionActive()) return;
    if (visionSession.state != VISION_STATE_IDLE &&
        visionSession.state != VISION_STATE_PICK_DONE) return;
    if (!mechanismState.valid || mechanismState.running ||
        mechanismState.current.horizontalDmm != 0 ||
        mechanismState.current.liftDmm != 0U ||
        mechanismState.current.turretDdeg != 2700U) {
        visionPickActive = 1U;
        failVision("PICK_REQUIRES_INITIAL_POSE");
        return;
    }
    color = rawPickColors[rawPickItemIndex];
    if (!vision_session_start(&visionSession, VISION_MODE_MATERIAL,
                              color, 0U, &visionMaterialCalibration, clockMs)) {
        visionPickActive = 1U;
        failVision("PICK_START_REJECTED");
        return;
    }
    visionPickActive = 1U;
    visionSession.state = VISION_STATE_PREP;
    serialSendString("ROUTE RAWPICK state=PREP color=");
    serialSendUint(color);
    serialSendString(" slot=");
    serialSendUint((uint16_t)(rawPickItemIndex + 1U));
    serialSendString("\r\n");
    startMechanismPose(&rawPickObservePose);
    if (!mechanismState.running) failVision("PICK_PREP_FAILED");
}

static void serviceVision(void)
{
    const VisionCalibration *calibration;
    VisionPacket packet;
    VisionEvent event;
    int16_t du;
    int16_t dv;
    if (!visionSessionActive()) {
        visionRxReady = 0U;
        return;
    }
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        failVision("IMU_OR_CAN_FAULT");
        return;
    }
    if (visionSession.state == VISION_STATE_PREP) {
        uint8_t color = visionSession.selector;
        if (!mechanismState.valid) {
            failVision("PICK_PREP_FAILED");
        } else if (!mechanismState.running) {
            if (mechanismState.current.horizontalDmm != -500 ||
                mechanismState.current.liftDmm != 0U ||
                mechanismState.current.turretDdeg != 2700U ||
                !vision_session_start(&visionSession, VISION_MODE_MATERIAL,
                                      color, 0U, visionCalibration(),
                                      clockMs)) {
                failVision("PICK_PREP_FAILED");
            } else {
                printVisionState();
                if (rawPickAtStation) {
                    serialSendString("ROUTE RAWPICK state=DETECT color=");
                    serialSendUint(color);
                    serialSendString(" slot=");
                    serialSendUint((uint16_t)(rawPickItemIndex + 1U));
                    serialSendString("\r\n");
                }
            }
        }
        return;
    }
    if (visionSession.state == VISION_STATE_PICK) {
        if (!mechanismState.valid) {
            failVision("PICK_ACTION_FAILED");
        } else if (!mechanismActionActive) {
            visionPickActive = 0U;
            visionSession.state = VISION_STATE_PICK_DONE;
            printVisionState();
            serialSendString("VISION PICK DONE color=");
            serialSendUint(visionSession.selector);
            serialSendString("\r\n");
            if (rawPickAtStation) {
                serialSendString("ROUTE RAWPICK state=ITEM_DONE color=");
                serialSendUint(rawPickColors[rawPickItemIndex]);
                serialSendString(" slot=");
                serialSendUint((uint16_t)(rawPickItemIndex + 1U));
                serialSendString("\r\n");
                ++rawPickItemIndex;
                rawPickMissingReported = 0U;
                if (rawPickItemIndex == 3U) {
                    serialSendString("ROUTE RAWPICK state=DONE color=6 slot=3\r\n");
                    if (!missionActive)
                        serialSendString("WAIT: verify three materials; send route next\r\n");
                } else {
                    serialSendString("ROUTE RAWPICK state=WAIT_MATERIAL color=");
                    serialSendUint(rawPickColors[rawPickItemIndex]);
                    serialSendString(" slot=");
                    serialSendUint((uint16_t)(rawPickItemIndex + 1U));
                    serialSendString("\r\n");
                }
            }
        }
        return;
    }
    calibration = visionCalibration();
    event = vision_session_tick(&visionSession, clockMs);
    if (event == VISION_EVENT_FAILED) {
        failVision(visionSession.fault);
        return;
    }
    if (visionSession.state == VISION_STATE_REQUEST) {
        vision_session_make_request(&visionSession, calibration, &packet, clockMs);
        visionUartSend(&packet);
        printVisionState();
    }
    if (!visionRxReady) return;
    __disable_irq();
    packet = visionRxPacket;
    visionRxReady = 0U;
    __enable_irq();
    if (rawPickAtStation &&
        vision_session_wait_for_material(&visionSession, &packet)) {
        if (!rawPickMissingReported) {
            serialSendString("ROUTE RAWPICK state=WAIT_MATERIAL color=");
            serialSendUint(rawPickColors[rawPickItemIndex]);
            serialSendString(" slot=");
            serialSendUint((uint16_t)(rawPickItemIndex + 1U));
            serialSendString("\r\n");
            rawPickMissingReported = 1U;
        }
        return;
    }
    event = vision_session_observe(&visionSession, calibration, &packet, clockMs);
    if (event == VISION_EVENT_NONE) return;
    du = (int16_t)(packet.value[0] - calibration->anchorU);
    dv = (int16_t)(packet.value[1] - calibration->anchorV);
    serialSendString("VISION SAMPLE token=");
    serialSendUint32(packet.token);
    serialSendString(" du="); serialSendInt(du);
    serialSendString(" dv="); serialSendInt(dv);
    serialSendString(" forward_mm="); serialSendFloat1(visionSession.forwardMm);
    serialSendString(" right_mm="); serialSendFloat1(visionSession.rightMm);
    serialSendString(" quality="); serialSendUint((uint16_t)packet.value[2]);
    serialSendString(" iteration="); serialSendUint(visionSession.iteration);
    serialSendString("\r\n");
    if (event == VISION_EVENT_MOVE) {
        if (!imuValid || clockMs - imuStamp > 250U ||
            !startVisionMotion(visionSession.forwardMm, visionSession.rightMm,
                               15U, 1U)) {
            failVision("MOTION_START_FAILED");
            return;
        }
        printVisionState();
    } else if (event == VISION_EVENT_ALIGNED) {
        if (visionPickActive) {
            const MechanismAction *actions = rawPickActions;
            if (rawPickAtStation) {
                rawPickActionsForSlot(rawPickRouteActions,
                                      (uint8_t)(rawPickItemIndex + 1U));
                actions = rawPickRouteActions;
            }
            if (!mechanismActionStart(&rawPickInitial, actions,
                                      RAW_PICK_ACTION_COUNT)) {
                failVision("PICK_START_FAILED");
            } else {
                visionSession.state = VISION_STATE_PICK;
                printVisionState();
                if (rawPickAtStation) {
                    serialSendString("ROUTE RAWPICK state=PICK color=");
                    serialSendUint(rawPickColors[rawPickItemIndex]);
                    serialSendString(" slot=");
                    serialSendUint((uint16_t)(rawPickItemIndex + 1U));
                    serialSendString("\r\n");
                }
            }
        } else {
            stopDriveMotors();
            serialSendString("VISION DONE forward_mm=");
            serialSendFloat1(visionSession.forwardMm);
            serialSendString(" right_mm=");
            serialSendFloat1(visionSession.rightMm);
            serialSendString(" iterations=");
            serialSendUint(visionSession.iteration);
            serialSendString("\r\n");
        }
    } else if (event == VISION_EVENT_FAILED) {
        failVision(visionSession.fault);
    }
}

static void missionReport(const char *phase)
{
    serialSendString("ROUTE MISSION phase=");
    serialSendString(phase);
    serialSendString(" station=");
    if (routeIndex == 2U) serialSendString("QR");
    else if (routeIndex == 4U) serialSendString("RAW_1");
    else if (routeIndex == 5U) serialSendString("COARSE_1");
    else if (routeIndex == 7U) serialSendString("TEMP_1");
    else if (routeIndex == 9U) serialSendString("RAW_2");
    else if (routeIndex == 10U) serialSendString("COARSE_2");
    else if (routeIndex == 12U) serialSendString("TEMP_2");
    else serialSendString("TRANSIT");
    serialSendString("\r\n");
}

static void missionAbort(const char *reason)
{
    if (!missionActive) return;
    stopServoMotion();
    stopAllMotors();
    serialSendString("ROUTE INVALID reason=mission_");
    serialSendString(reason);
    serialSendString("\r\n");
}

static void missionContinue(void)
{
    rawPickAtStation = 0U;
    visionPickActive = 0U;
    vision_session_init(&visionSession);
    missionPhase = MISSION_IDLE;
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    routeWaiting = 0U;
    routeTick = routeSettle = routeLegStart = clockMs;
    missionReport("CONTINUE");
}

static void missionBeginStation(uint8_t nextIndex)
{
    missionActionStarted = 0U;
    missionOffsetForward = missionOffsetRight = 0.0f;
    vision_session_init(&visionSession);
    if (nextIndex == 2U) {
        missionPhase = MISSION_QR_WAIT;
        missionDeadline = clockMs + 1000U;
        missionReport("QR_WAIT");
    } else if (nextIndex == 4U || nextIndex == 9U) {
        missionPhase = MISSION_RAW_PICK;
        rawPickAtStation = 1U;
        rawPickItemIndex = rawPickMissingReported = missionObservedItem = 0U;
        missionDeadline = clockMs + 30000U;
        missionReport("RAW_PICK");
    } else if (nextIndex == 5U || nextIndex == 10U) {
        missionPhase = MISSION_RING_ALIGN;
        missionReport("RING_ALIGN");
    } else if (nextIndex == 7U || nextIndex == 12U) {
        missionPhase = MISSION_TEMP_ALIGN;
        missionBuildStoragePlace(missionWorkActions,
                                 nextIndex == 12U ? 2U : 1U);
        missionReport("TEMP_ALIGN");
    }
}

static void serviceMission(void)
{
    float forward, right;
    if (!missionActive || !routeActive || !routeWaiting) return;
    if (!imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        missionAbort("imu_or_can");
        return;
    }
    if (missionPhase == MISSION_QR_WAIT) {
        if ((int32_t)(clockMs - missionDeadline) >= 0) missionContinue();
        return;
    }
    if (missionPhase == MISSION_RAW_PICK) {
        if (rawPickItemIndex != missionObservedItem) {
            missionObservedItem = rawPickItemIndex;
            missionDeadline = clockMs + 30000U;
        }
        if (rawPickItemIndex == 3U && !visionPickActive &&
            !mechanismActionActive) {
            missionPhase = MISSION_RECENTER;
            missionReport("RECENTER");
        } else if ((int32_t)(clockMs - missionDeadline) >= 0) {
            missionAbort("material_timeout");
        }
        return;
    }
    if (missionPhase == MISSION_RING_ALIGN ||
        missionPhase == MISSION_TEMP_ALIGN) {
        if (!missionActionStarted) {
            if (!mechanismState.valid || mechanismState.running ||
                mechanismState.current.liftDmm != 0U ||
                mechanismState.current.turretDdeg != 2700U) {
                missionAbort("ring_prep_pose");
                return;
            }
            startMechanismPose(&missionRingObservePose);
            if (!mechanismState.running) {
                missionAbort("ring_prep_start");
                return;
            }
            missionActionStarted = 1U;
            missionReport("RING_PREP");
        } else if (missionActionStarted == 1U && !mechanismState.running) {
            if (!mechanismState.valid ||
                mechanismState.current.horizontalDmm != -500 ||
                mechanismState.current.liftDmm != 0U ||
                mechanismState.current.turretDdeg != 2700U) {
                missionAbort("ring_prep_failed");
                return;
            }
            if (!vision_session_start(&visionSession, VISION_MODE_RING, 0U,
                                      2U, &visionRingCalibration[1], clockMs)) {
                missionAbort("ring_start");
                return;
            }
            missionActionStarted = 2U;
            printVisionState();
        } else if (missionActionStarted == 2U &&
                   visionSession.state == VISION_STATE_ALIGNED) {
            missionPhase = missionPhase == MISSION_TEMP_ALIGN ?
                           MISSION_TEMP_PLACE : MISSION_COARSE_PLACE;
            missionActionStarted = 0U;
            missionReport(missionPhase == MISSION_TEMP_PLACE ?
                          "TEMP_PLACE" : "COARSE_PLACE");
        }
        return;
    }
    if (missionPhase == MISSION_COARSE_PLACE ||
        missionPhase == MISSION_COARSE_PICK ||
        missionPhase == MISSION_TEMP_PLACE) {
        if (!missionActionStarted) {
            const MechanismAction *actions = missionWorkActions;
            uint16_t count = MISSION_PLACE_COUNT;
            if (missionPhase == MISSION_COARSE_PLACE)
                actions = missionPlaceActions;
            else if (missionPhase == MISSION_COARSE_PICK) {
                missionBuildPickBack(missionPickBackActions);
                actions = missionPickBackActions;
                count = MISSION_PICK_BACK_COUNT;
            }
            if (!mechanismActionStart(&missionInitial, actions, count)) {
                missionAbort("action_start");
                return;
            }
            missionActionStarted = 1U;
        } else if (!mechanismActionActive) {
            if (!mechanismState.valid ||
                mechanismActionIndex != mechanismActionCount) {
                missionAbort("action_failed");
                return;
            }
            if (missionPhase == MISSION_COARSE_PLACE) {
                missionPhase = MISSION_COARSE_PICK;
                missionActionStarted = 0U;
                missionReport("COARSE_PICK");
            } else {
                missionPhase = MISSION_RECENTER;
                missionReport("RECENTER");
            }
        }
        return;
    }
    if (missionPhase != MISSION_RECENTER) return;
    if (motionMode || mechanismState.running || mechanismActionActive) return;
    if (!mechanismState.valid || mechanismState.current.horizontalDmm != 0 ||
        mechanismState.current.liftDmm != 0U ||
        mechanismState.current.turretDdeg != 2700U) {
        missionAbort("unsafe_recenter_pose");
        return;
    }
    if (routeAbs(missionOffsetForward) <= 1.0f &&
        routeAbs(missionOffsetRight) <= 1.0f) {
        missionContinue();
        return;
    }
    forward = -missionOffsetForward;
    right = -missionOffsetRight;
    if (forward > 20.0f) forward = 20.0f;
    if (forward < -20.0f) forward = -20.0f;
    if (right > 20.0f) right = 20.0f;
    if (right < -20.0f) right = -20.0f;
    if (!startVisionMotion(forward, right, 15U, 0U))
        missionAbort("recenter_failed");
}

static uint8_t processVisionCommand(const char *command)
{
    const char *cursor;
    const VisionCalibration *calibration;
    uint16_t value;
    uint16_t rpm;
    uint16_t forwardMilli;
    uint16_t rightMilli;
    int16_t forward;
    int16_t right;
    uint8_t mode;
    uint8_t selector;
    uint8_t target;
    if (strcmp(command, "vision status") == 0) {
        printVisionState();
        return 1U;
    }
    if (strcmp(command, "vision pause") == 0) {
        if (visionPickActive) {
            uint8_t wasRawPick = rawPickAtStation;
            stopServoMotion();
            stopAllMotors();
            if (wasRawPick) serialSendString("ROUTE INVALID reason=rawpick_paused\r\n");
        }
        if (motionMode == 4U) {
            stopDriveMotors();
            motionMode = 0U;
        }
        vision_session_pause(&visionSession);
        serialSendString("VISION PAUSED\r\n");
        return 1U;
    }
    if (visionSessionActive() && strncmp(command, "vision ", 7U) == 0) {
        failVision("COMMAND_CONFLICT");
        return 1U;
    }
    if (strncmp(command, "vision pick material ", 21U) == 0) {
        cursor = command + 21U;
        if (!mechanismParseUnsigned(&cursor, &value) ||
            !mechanismAtEnd(cursor) || value < 1U || value > 6U) {
            serialSendString("VISION ERROR reason=MATERIAL_COLOR_RANGE\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running ||
            mechanismActionActive || visionSessionActive()) {
            serialSendString("VISION ERROR reason=PICK_BUSY\r\n");
            return 1U;
        }
        if (!armed) {
            serialSendString("VISION ERROR reason=PICK_NOT_ARMED\r\n");
            return 1U;
        }
        if (!mechanismState.valid) {
            serialSendString("VISION ERROR reason=PICK_MECH_NOT_INITIALIZED\r\n");
            return 1U;
        }
        if (!imuValid || clockMs - imuStamp > 250U) {
            serialSendString("VISION ERROR reason=PICK_IMU_STALE\r\n");
            return 1U;
        }
        if (motionInterrupted()) {
            serialSendString("VISION ERROR reason=PICK_CAN_OR_ESTOP\r\n");
            return 1U;
        }
        if (servoChannelMoving[0] || servoChannelMoving[1] ||
            servoChannelMoving[2]) {
            serialSendString("VISION ERROR reason=PICK_SERVO_MOVING\r\n");
            return 1U;
        }
        if (mechanismState.current.horizontalDmm != 0 ||
            mechanismState.current.liftDmm != 0U ||
            mechanismState.current.turretDdeg != 2700U) {
            serialSendString("VISION ERROR reason=PICK_REQUIRES_INITIAL_POSE\r\n");
            return 1U;
        }
        if (!vision_session_start(&visionSession, VISION_MODE_MATERIAL,
                                  (uint8_t)value, 0U,
                                  &visionPickCalibration, clockMs)) {
            serialSendString("VISION ERROR reason=PICK_START_REJECTED\r\n");
            return 1U;
        }
        armed = 0U;
        visionPickActive = 1U;
        visionSession.state = VISION_STATE_PREP;
        startMechanismPose(&rawPickObservePose);
        if (!mechanismState.running) {
            failVision("PICK_PREP_FAILED");
        } else {
            printVisionState();
        }
        return 1U;
    }
    if (strncmp(command, "vision scale material ", 22U) == 0) {
        cursor = command + 22U;
        if (!mechanismParseUnsigned(&cursor, &forwardMilli) ||
            !mechanismParseUnsigned(&cursor, &rightMilli) ||
            !mechanismAtEnd(cursor) ||
            forwardMilli < 50U || forwardMilli > 2000U ||
            rightMilli < 50U || rightMilli > 2000U) {
            serialSendString("VISION ERROR reason=SCALE_RANGE\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running ||
            mechanismActionActive || visionSession.state == VISION_STATE_PAUSED ||
            !armed || motionInterrupted()) {
            serialSendString("VISION ERROR reason=SCALE_REQUIRES_ARMED_IDLE\r\n");
            return 1U;
        }
        armed = 0U;
        visionMaterialCalibration.anchorU = 160;
        visionMaterialCalibration.anchorV = 120;
        visionMaterialCalibration.matrix[0] = -(float)forwardMilli / 1000.0f;
        visionMaterialCalibration.matrix[1] = 0.0f;
        visionMaterialCalibration.matrix[2] = 0.0f;
        visionMaterialCalibration.matrix[3] = -(float)rightMilli / 1000.0f;
        visionMaterialCalibration.calibrated = 1U;
        serialSendString("VISION SCALE material forward_milli=");
        serialSendUint(forwardMilli);
        serialSendString(" right_milli=");
        serialSendUint(rightMilli);
        serialSendString("\r\n");
        return 1U;
    }
    if (strncmp(command, "vision scale ring ", 18U) == 0) {
        cursor = command + 18U;
        if (!mechanismParseUnsigned(&cursor, &value) ||
            !mechanismParseUnsigned(&cursor, &forwardMilli) ||
            !mechanismParseUnsigned(&cursor, &rightMilli) ||
            !mechanismAtEnd(cursor) || value < 1U || value > 3U ||
            forwardMilli < 50U || forwardMilli > 2000U ||
            rightMilli < 50U || rightMilli > 2000U) {
            serialSendString("VISION ERROR reason=SCALE_RANGE\r\n");
            return 1U;
        }
        if (motionMode || routeActive || mechanismState.running ||
            mechanismActionActive || visionSession.state == VISION_STATE_PAUSED ||
            !armed || motionInterrupted()) {
            serialSendString("VISION ERROR reason=SCALE_REQUIRES_ARMED_IDLE\r\n");
            return 1U;
        }
        armed = 0U;
        visionRingCalibration[value - 1U].anchorU = 160;
        visionRingCalibration[value - 1U].anchorV = 120;
        visionRingCalibration[value - 1U].matrix[0] = -(float)forwardMilli / 1000.0f;
        visionRingCalibration[value - 1U].matrix[1] = 0.0f;
        visionRingCalibration[value - 1U].matrix[2] = 0.0f;
        visionRingCalibration[value - 1U].matrix[3] = -(float)rightMilli / 1000.0f;
        visionRingCalibration[value - 1U].calibrated = 1U;
        serialSendString("VISION SCALE ring=");
        serialSendUint(value);
        serialSendString(" forward_milli=");
        serialSendUint(forwardMilli);
        serialSendString(" right_milli=");
        serialSendUint(rightMilli);
        serialSendString("\r\n");
        return 1U;
    }
    if (strncmp(command, "vision jog ", 11U) == 0) {
        cursor = command + 11U;
        if (!mechanismParseSigned(&cursor, &forward) ||
            !mechanismParseSigned(&cursor, &right) ||
            !mechanismParseUnsigned(&cursor, &rpm) || !mechanismAtEnd(cursor) ||
            (forward == 0 && right == 0) || forward < -20 || forward > 20 ||
            right < -20 || right > 20 || rpm < 10U || rpm > 30U) {
            serialSendString("VISION ERROR reason=JOG_RANGE\r\n");
            return 1U;
        }
        if (visionSession.state != VISION_STATE_PAUSED || motionMode || routeActive ||
            mechanismState.running || mechanismActionActive || !armed ||
            !imuValid || clockMs - imuStamp > 250U) {
            serialSendString("VISION ERROR reason=JOG_REQUIRES_PAUSED_ARMED_IDLE_FRESH_IMU\r\n");
            return 1U;
        }
        armed = 0U;
        if (!startVisionMotion((float)forward, (float)right, rpm, 0U)) {
            failVision("JOG_START_FAILED");
        } else {
            serialSendString("VISION STATE state=MOVE mode=MANUAL selector=0 target=0 iteration=0\r\n");
        }
        return 1U;
    }
    if (strncmp(command, "vision align material ", 22U) == 0) {
        cursor = command + 22U;
        mode = VISION_MODE_MATERIAL;
        selector = 0U;
        target = 0U;
        if (!mechanismParseUnsigned(&cursor, &value) || !mechanismAtEnd(cursor) ||
            value < 1U || value > 6U) {
            serialSendString("VISION ERROR reason=MATERIAL_COLOR_RANGE\r\n");
            return 1U;
        }
        selector = (uint8_t)value;
        calibration = &visionMaterialCalibration;
    } else if (strncmp(command, "vision align ring ", 18U) == 0) {
        cursor = command + 18U;
        mode = VISION_MODE_RING;
        selector = 0U;
        target = 0U;
        if (!mechanismParseUnsigned(&cursor, &value) || !mechanismAtEnd(cursor) ||
            value < 1U || value > 3U) {
            serialSendString("VISION ERROR reason=RING_RANGE\r\n");
            return 1U;
        }
        target = (uint8_t)value;
        calibration = &visionRingCalibration[target - 1U];
    } else {
        return 0U;
    }
    if (!vision_calibration_valid(calibration)) {
        serialSendString("VISION ERROR reason=CALIBRATION_INVALID\r\n");
        return 1U;
    }
    if (motionMode || routeActive || mechanismState.running ||
        mechanismActionActive || visionSessionActive() || !armed ||
        !imuValid || clockMs - imuStamp > 250U || motionInterrupted()) {
        serialSendString("VISION ERROR reason=ALIGN_REQUIRES_ARMED_IDLE_FRESH_IMU\r\n");
        return 1U;
    }
    if (!vision_session_start(&visionSession, mode, selector, target,
                              calibration, clockMs)) {
        serialSendString("VISION ERROR reason=START_REJECTED\r\n");
        return 1U;
    }
    armed = 0U;
    invalidateNavigation("vision_motion");
    printVisionState();
    return 1U;
}

static void processCommand(const char *command)
{
    uint16_t id, value;
    if (emergencyStop) return;
    if (strcmp(command, "hb") == 0) {
        hostHeartbeatActive = 1U;
        hostHeartbeatStamp = clockMs;
        return;
    }
    if (processVisionCommand(command)) return;
    if (visionSessionActive() && strcmp(command, "stop") != 0 &&
        strcmp(command, "X") != 0 && strcmp(command, "x") != 0 &&
        strcmp(command, "disable") != 0) {
        serialSendString("ERR: vision active; use vision pause, stop or ! first\r\n");
        return;
    }
    if (processMechanismCommand(command)) return;
    if (processNavCommand(command)) return;
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
    if (processAuxMoveCommand(command)) return;
    if (strncmp(command, "line ", 5U) == 0) {
        startLine(command + 5, 0U);
        return;
    }
    if (strncmp(command, "straight ", 9U) == 0) {
        startLine(command + 9, 1U);
        return;
    }
    if (strncmp(command, "pid move ", 9U) == 0) {
        startLine(command + 9, 2U);
        return;
    }
    if (strcmp(command, "status") == 0) { printStatus(); return; }
    if (strncmp(command, "cancheck ", 9U) == 0) {
        const char *cursor = command + 9;
        if (!parseUint(&cursor, &id) || *cursor != '\0' || id < 1U || id > 6U) {
            serialSendString("ERR: cancheck 1..6\r\n");
        } else checkMotorCan((uint8_t)id);
        return;
    }
    if (strncmp(command, "motor ", 6U) == 0) {
        if (!parsePair(command + 6, &id, &value) ||
            id < AUX_MOTOR_MIN_ID || id > AUX_MOTOR_MAX_ID || value > 1U) {
            serialSendString("ERR: motor 5|6 0|1\r\n");
            return;
        }
        invalidateMechanism("manual_jog");
        startAuxMotorJog((uint8_t)id, (uint8_t)value);
        return;
    }
    if (strcmp(command, "imu 9600") == 0 || strcmp(command, "imu 115200") == 0) {
        invalidateNavigation("imu_reconfigured");
        hwt101Init(strcmp(command, "imu 9600") == 0 ? 9600U : 115200U);
        serialSendString("IMU receiver configured; sensor settings unchanged\r\n");
        return;
    }
    if (strcmp(command, "yawdir 0") == 0 || strcmp(command, "yawdir 1") == 0) {
        invalidateNavigation("yaw_direction_changed");
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
            invalidateNavigation("manual_motion");
            Emm_V5_En_Control((uint8_t)id, true, false);
            delay_ms(2U);
            if (motionInterrupted()) { stopAllMotors(); return; }
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
    } else if (strcmp(command, "disable") == 0) {
        uint8_t hadFullRoute = fullRouteRunning;
        uint8_t hadVision = visionSessionActive() || motionMode == 4U;
        if (visionPickActive) stopServoMotion();
        stopAllMotors();
        if (hadFullRoute) serialSendString("ROUTE INVALID reason=disabled\r\n");
        if (hadVision) serialSendString("VISION PAUSED\r\n");
        setAllMotorsEnabled(false);
        setAuxMotorsEnabled(false);
        armed = 0U;
        serialSendString("OK: motors 1..6 disabled and disarmed\r\n");
    } else if (strncmp(command, "servo ", 6U) == 0) {
        if (command[6] == '4') invalidateMechanism("turret_manual");
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
        uint8_t hadNavigation = navInitialized;
        uint8_t hadFullRoute = fullRouteRunning;
        uint8_t hadVision = visionSessionActive() || motionMode == 4U;
        stopServoMotion();
        stopAllMotors();
        if (hadNavigation) serialSendString("NAV INVALID reason=stopped\r\n");
        if (hadFullRoute) serialSendString("ROUTE INVALID reason=stopped\r\n");
        if (hadVision) serialSendString("VISION PAUSED\r\n");
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
    hwt101Init(115200U);
    visionUartInit();

    mechanismStateReset(&mechanismState);
    vision_session_init(&visionSession);
    stopAllMotors();
    serialSendString("\r\nYYB mecanum/servo jog ready; motors 1..6 stopped; disarmed.\r\n");
    printHelp();

    for (;;) {
        if (jogCanFault) {
            uint8_t hadNavigation = navInitialized;
            uint8_t hadFullRoute = fullRouteRunning;
            uint8_t hadVision = visionSessionActive() || motionMode == 4U;
            if (visionPickActive) stopServoMotion();
            stopAllMotors();
            jogCanFault = 0U;
            if (hadNavigation) serialSendString("NAV INVALID reason=can_fault\r\n");
            if (hadFullRoute) serialSendString("ROUTE INVALID reason=can_fault\r\n");
            if (hadVision) failVision("CAN_FAULT");
            serialSendString("ERR: CAN transmit failed; stop attempted, check power/bus\r\n");
        }
        if (emergencyStop) {
            uint8_t hadNavigation = navInitialized;
            uint8_t hadFullRoute = fullRouteRunning;
            uint8_t hadVision = visionSessionActive() || motionMode == 4U;
            stopServoMotion();
            stopAllMotors();
            __disable_irq();
            emergencyStop = 0U;
            rxLength = 0U;
            rxReady = 0U;
            __enable_irq();
            if (hadNavigation) serialSendString("NAV INVALID reason=emergency_stop\r\n");
            if (hadFullRoute) serialSendString("ROUTE INVALID reason=emergency_stop\r\n");
            if (hadVision) failVision("EMERGENCY_STOP");
            serialSendString("EMERGENCY STOP; motors 1..6 stopped; disarmed\r\n");
        }

        if (rxReady) {
            __disable_irq();
            strcpy(command, (const char *)rxLine);
            rxLength = 0U;
            rxReady = 0U;
            __enable_irq();
            processCommand(command);
        }
        serviceHostWatchdog();
        serviceMotion();
        serviceVision();
        serviceMechanism();
        mechanismActionService();
        serviceRoute();
        serviceRawPickRoute();
        serviceMission();
        reportCompletedServoMoves();
    }
}
