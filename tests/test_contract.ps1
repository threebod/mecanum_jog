$ErrorActionPreference = 'Stop'

$exampleRoot = Split-Path -Parent $PSScriptRoot
$mainPath = Join-Path $exampleRoot 'main.c'
$projectPath = Join-Path $exampleRoot 'MecanumJog.uvprojx'
$readmePath = Join-Path $exampleRoot 'README.md'
$navigationPath = Join-Path $exampleRoot 'map_navigation.h'
$sensorPath = Join-Path $exampleRoot 'hwt101.c'
$canPath = Join-Path $exampleRoot 'jog_can_diagnostics.c'

foreach ($path in @($mainPath, $projectPath, $readmePath, $navigationPath, $sensorPath, $canPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing required file: $path"
    }
}

$main = Get-Content -LiteralPath $mainPath -Raw
$project = Get-Content -LiteralPath $projectPath -Raw
$sensor = Get-Content -LiteralPath $sensorPath -Raw
$canSource = Get-Content -LiteralPath $canPath -Raw

$requiredMainPatterns = @(
    '#define COMMAND_BAUD_RATE\s+115200U',
    '#define RX_LINE_SIZE\s+80U',
    '#define MOTOR_TEST_SPEED\s+30U',
    '#define MOTOR_TEST_ACCEL\s+50U',
    '#define MOTOR_TEST_PULSES\s+160U',
    '#define AUX_MOTOR_MIN_ID\s+5U',
    '#define AUX_MOTOR_MAX_ID\s+6U',
    '#define AUX_MOVE_STATUS_POLL_MS\s+50U',
    '#define AUX_MOVE_TIMEOUT_MARGIN_MS\s+3000U',
    '#define HOST_HEARTBEAT_TIMEOUT_MS\s+1000U',
    '#define SERVO_MIN_PULSE_US\s+500U',
    '#define SERVO_PULSE_RANGE_US\s+2000U',
    '#define SERVO_PERIOD_US\s+20000U',
    '#define SERVO_DEFAULT_SPEED_DPS10\s+1200U',
    '#define SERVO_MAX_SPEED_DPS10\s+1800U',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_UART5, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOC, GPIO_PinSource12, GPIO_AF_UART5\)',
    'GPIO_PinAFConfig\(GPIOD, GPIO_PinSource2, GPIO_AF_UART5\)',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_UART4, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOC, GPIO_PinSource10, GPIO_AF_UART4\)',
    'GPIO_PinAFConfig\(GPIOC, GPIO_PinSource11, GPIO_AF_UART4\)',
    'void UART5_IRQHandler\(void\)',
    'void UART4_IRQHandler\(void\)',
    'void TIM2_IRQHandler\(void\)',
    '(?s)static void serviceHostWatchdog\(void\).*?hostHeartbeatActive = 0U;.*?stopServoMotion\(\);.*?stopAllMotors\(\);.*?ERR: host heartbeat timeout; stopped',
    '(?s)static void processCommand\(const char \*command\).*?strcmp\(command, "hb"\) == 0.*?hostHeartbeatActive = 1U;.*?hostHeartbeatStamp = clockMs;.*?return;.*?processRouteCommand\(command\)',
    '(?s)for \(;;\).*?serviceHostWatchdog\(\);.*?serviceMotion\(\);',
    '(?s)for \(;;\).*?serviceMotion\(\);.*?serviceVision\(\);',
    '(?s)strncmp\(command, "pid move ", 9U\) == 0.*?startLine\(command \+ 9, 2U\)',
    '#include "vision_control.h"',
    '#include "vision_config.h"',
    '(?s)static uint8_t processVisionCommand\(const char \*command\).*?vision pause.*?vision jog .*?vision align material .*?vision align ring ',
    '(?s)visionSessionActive\(\).*?vision .*?failVision\("COMMAND_CONFLICT"\)',
    '(?s)if \(visionSessionActive\(\).*?ERR: vision active; use vision pause, stop or ! first',
    "received == '!'",
    'else if \(!emergencyStop &&',
    '(?s)if \(emergencyStop\)\s*\{.*?stopServoMotion\(\);.*?stopAllMotors\(\);.*?__disable_irq\(\);.*?emergencyStop = 0U;',
    'static void setAllMotorsEnabled\(bool enabled\)',
    'Emm_V5_En_Control\(id, enabled, false\)',
    '(?s)strcmp\(command, "disable"\) == 0.*?setAllMotorsEnabled\(false\).*?armed = 0U;',
    '(?s)static void startAuxMotorJog\(uint8_t motorId, uint8_t direction\).*?Emm_V5_En_Control\(motorId, true, false\).*?Emm_V5_Pos_Control\(motorId, direction,.*?false, false\)',
    '(?s)static uint8_t processAuxMoveCommand\(const char \*command\).*?motorId < AUX_MOTOR_MIN_ID.*?motorId > AUX_MOTOR_MAX_ID.*?maximumDmm = motorId == 5U \? 1500 : 1870;.*?distanceDmm == 0.*?mechanismLiftPulses.*?mechanismHorizontalPulses.*?Emm_V5_Pos_Control\(\(uint8_t\)motorId, direction, rpm,.*?accel, pulses, false, false\).*?motionMode = 3U.*?motionDuration = mechanismMotorDurationMs\(pulses, rpm\) \* 2U \+.*?AUX_MOVE_TIMEOUT_MARGIN_MS',
    '(?s)static void serviceMotion\(void\).*?if \(motionMode == 3U\).*?function == 0x3AU.*?status & 0x02U.*?DONE: auxmove motor=.*?Emm_V5_Read_Sys_Params\(auxMoveMotorId, S_FLAG\)',
    '(?s)static void processCommand\(const char \*command\).*?processAuxMoveCommand\(command\)',
    '(?s)strncmp\(command, "motor ", 6U\) == 0.*?id < AUX_MOTOR_MIN_ID.*?id > AUX_MOTOR_MAX_ID.*?startAuxMotorJog',
    '(?s)if \(wheel\).*?Emm_V5_En_Control\(\(uint8_t\)id, true, false\).*?Emm_V5_Pos_Control',
    '(?s)static void checkMotorCan\(uint8_t motorId\).*?jogCanReadStatus\(motorId, &emergencyStop, &reply\)',
    'CAN RX motor ',
    ' no CAN reply; ESR=',
    'CAN1->ESR',
    'CAN1->TSR',
    'TX queued: chassis ',
    'TX queued: motor ',
    '(?s)static void stopDriveMotors\(void\).*?id <= MOTOR_MAX_ID.*?Emm_V5_Stop_Now\(id, false\)',
    '(?s)static void stopAuxMotors\(void\).*?id <= AUX_MOTOR_MAX_ID.*?Emm_V5_Stop_Now\(id, false\)',
    '(?s)static void stopAllMotors\(void\).*?stopDriveMotors\(\);.*?stopAuxMotors\(\)',
    '#include "mechanism_action.h"',
    'static MechanismState mechanismState',
    '(?s)static uint8_t processMechanismCommand\(const char \*commandText\).*?mechanismParseCommand.*?MECHANISM_COMMAND_INIT.*?MECHANISM_COMMAND_POSE',
    '(?s)static void startMechanismPose\(const MechanismPose \*target\).*?Emm_V5_Pos_Control\(6U,.*?true\).*?Emm_V5_Pos_Control\(5U,.*?true\).*?Emm_V5_Synchronous_motion\(0x00\)',
    '(?s)static void serviceMechanism\(void\).*?MECHANISM_EVENT_POSITION.*?MECHANISM_EVENT_DONE',
    '(?s)uint8_t mechanismActionStart\(const MechanismInitialState \*initial,.*?const MechanismAction \*actions,.*?uint16_t count\)',
    '(?s)void mechanismActionService\(void\).*?MECHANISM_ACTION_POSE.*?MECHANISM_ACTION_GRIPPER.*?MECHANISM_ACTION_PLATFORM.*?MECHANISM_ACTION_SERVO.*?MECHANISM_ACTION_WAIT',
    '(?s)for \(;;\).*?serviceMechanism\(\);',
    '(?s)for \(;;\).*?mechanismActionService\(\);',
    '(?s)strncmp\(command, "motor ", 6U\) == 0.*?invalidateMechanism\("manual_jog"\)',
    '(?s)static uint8_t processNavCommand\(const char \*command\).*?nav init 1.*?nav init 2.*?nav goto .*?navPointClear.*?navPlanPath',
    '(?s)static void printFullRoutePose\(const char \*state\).*?ROUTE POS x=.*?target_x=.*?target_y=',
    '(?s)static void finishFullRoute\(void\).*?ROUTE DONE x=',
    '(?s)route scale .*?degrees < 5000U.*?degrees > 15000U.*?routeLateralScale = degrees / 10000.0f',
    '(?s)route tune .*?routeForwardScale = forwardBp / 10000.0f.*?routeLateralScale = lateralBp / 10000.0f.*?routeTurnRpmLimit = rpm.*?routeLateralRpmLimit = lateralRpm',
    'Emm_V5_Synchronous_motion\(0x00\)',
    '(?s)static CommandResult parseServoCommand\(const char \*line,.*?channel < 2U \|\| channel > 4U.*?channel < 4U && angle > 270U.*?channel == 4U && angle > 360U.*?speedDps10 < 10U \|\| speedDps10 > SERVO_MAX_SPEED_DPS10',
    '(?s)strncmp\(command, "servo ", 6U\) == 0.*?processServoCommand\(command\)',
    '(?s)static void stopServoMotion\(void\).*?targetAngleMdeg\[index\] = currentAngleMdeg\[index\]',
    '(?s)if \(emergencyStop\).*?stopServoMotion\(\);.*?stopAllMotors\(\)',
    '(?s)strcmp\(command, "stop"\) == 0.*?stopServoMotion\(\);.*?stopAllMotors\(\)',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_TIM2, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOA, pinSource, GPIO_AF_TIM2\)',
    'TIM_ITConfig\(TIM2, TIM_IT_Update, ENABLE\)',
    '(?s)static const uint8_t motorDirections\[4\]\[5\] = \{\s*\{1U, 1U, 0U, 0U, 1U\},\s*\{0U, 0U, 1U, 1U, 0U\},\s*\{1U, 1U, 1U, 0U, 0U\},\s*\{0U, 0U, 0U, 1U, 1U\}',
    '(?s)wheelMm\[0\] = forward - right;\s*wheelMm\[1\] = forward \+ right;\s*wheelMm\[2\] = forward - right;\s*wheelMm\[3\] = forward \+ right;'
)

foreach ($pattern in $requiredMainPatterns) {
    if ($main -notmatch $pattern) {
        throw "main.c does not satisfy contract pattern: $pattern"
    }
}

foreach ($check in @(
    @{ Source = $sensor; Pattern = 'void USART2_IRQHandler\(void\)' },
    @{ Source = $sensor; Pattern = 'void hwt101Init\(uint32_t baud\)' },
    @{ Source = $canSource; Pattern = '#define CAN_CHECK_TIMEOUT_MS\s+300U' },
    @{ Source = $canSource; Pattern = 'uint8_t jogCanReadStatus\(' }
)) {
    if ($check.Source -notmatch $check.Pattern) {
        throw "Missing extracted module pattern: $($check.Pattern)"
    }
}

$forbiddenMainPatterns = @(
    'OK: chassis ',
    'OK: motor 5 direction ',
    'strcmp\(command, "enable"\) == 0',
    'strcmp\(command, "enable5"\) == 0',
    'strcmp\(command, "disable5"\) == 0'
)

foreach ($pattern in $forbiddenMainPatterns) {
    if ($main -match $pattern) {
        throw "main.c still contains wired USART1 configuration: $pattern"
    }
}

if ($project -notmatch '<Device>STM32F407ZG</Device>') {
    throw 'Keil target is not STM32F407ZG.'
}
if ($project -match 'STM32F103') {
    throw 'Keil project contains an STM32F103 target setting.'
}
if ($project -notmatch 'stm32f4xx_tim\.c') {
    throw 'Keil project does not include the TIM driver required by servo PWM.'
}

[xml]$projectXml = $project
$projectDirectory = Split-Path -Parent $projectPath
$filePaths = $projectXml.Project.Targets.Target.Groups.Group.Files.File.FilePath
foreach ($filePath in $filePaths) {
    $resolved = [System.IO.Path]::GetFullPath((Join-Path $projectDirectory $filePath))
    if (-not (Test-Path -LiteralPath $resolved -PathType Leaf)) {
        throw "Keil source path does not exist: $filePath"
    }
}

Write-Output 'PASS: mecanum jog example contract'
