$ErrorActionPreference = 'Stop'

$exampleRoot = Split-Path -Parent $PSScriptRoot
$mainPath = Join-Path $exampleRoot 'main.c'
$projectPath = Join-Path $exampleRoot 'MecanumJog.uvprojx'
$readmePath = Join-Path $exampleRoot 'README.md'
$navigationPath = Join-Path $exampleRoot 'map_navigation.h'

foreach ($path in @($mainPath, $projectPath, $readmePath, $navigationPath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing required file: $path"
    }
}

$main = Get-Content -LiteralPath $mainPath -Raw
$project = Get-Content -LiteralPath $projectPath -Raw

$requiredMainPatterns = @(
    '#define COMMAND_BAUD_RATE\s+115200U',
    '#define RX_LINE_SIZE\s+80U',
    '#define MOTOR_TEST_SPEED\s+30U',
    '#define MOTOR_TEST_ACCEL\s+50U',
    '#define MOTOR_TEST_PULSES\s+160U',
    '#define AUX_MOTOR_MIN_ID\s+5U',
    '#define AUX_MOTOR_MAX_ID\s+6U',
    '#define CAN_CHECK_TIMEOUT_MS\s+300U',
    '#define HOST_HEARTBEAT_TIMEOUT_MS\s+1000U',
    '#define SERVO_MIN_PULSE_US\s+500U',
    '#define SERVO_PULSE_RANGE_US\s+2000U',
    '#define SERVO_PERIOD_US\s+20000U',
    '#define SERVO_SPEED_DPS\s+30U',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_UART5, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOC, GPIO_PinSource12, GPIO_AF_UART5\)',
    'GPIO_PinAFConfig\(GPIOD, GPIO_PinSource2, GPIO_AF_UART5\)',
    'void UART5_IRQHandler\(void\)',
    'void TIM2_IRQHandler\(void\)',
    '(?s)static void serviceHostWatchdog\(void\).*?hostHeartbeatActive = 0U;.*?stopServoMotion\(\);.*?stopAllMotors\(\);.*?ERR: host heartbeat timeout; stopped',
    '(?s)static void processCommand\(const char \*command\).*?strcmp\(command, "hb"\) == 0.*?hostHeartbeatActive = 1U;.*?hostHeartbeatStamp = clockMs;.*?return;.*?processRouteCommand\(command\)',
    '(?s)for \(;;\).*?serviceHostWatchdog\(\);.*?serviceMotion\(\);',
    "received == '!'",
    'else if \(!emergencyStop &&',
    '(?s)if \(emergencyStop\)\s*\{.*?stopServoMotion\(\);.*?stopAllMotors\(\);.*?__disable_irq\(\);.*?emergencyStop = 0U;',
    'static void setAllMotorsEnabled\(bool enabled\)',
    'Emm_V5_En_Control\(id, enabled, false\)',
    '(?s)strcmp\(command, "disable"\) == 0.*?setAllMotorsEnabled\(false\).*?armed = 0U;',
    '(?s)static void startAuxMotorJog\(uint8_t motorId, uint8_t direction\).*?Emm_V5_En_Control\(motorId, true, false\).*?Emm_V5_Pos_Control\(motorId, direction,.*?false, false\)',
    '(?s)strncmp\(command, "motor ", 6U\) == 0.*?id < AUX_MOTOR_MIN_ID.*?id > AUX_MOTOR_MAX_ID.*?startAuxMotorJog',
    '(?s)if \(wheel\).*?Emm_V5_En_Control\(\(uint8_t\)id, true, false\).*?Emm_V5_Pos_Control',
    '(?s)static void checkMotorCan\(uint8_t motorId\).*?can\.rxFrameFlag = false;.*?Emm_V5_Read_Sys_Params\(motorId, S_FLAG\).*?CAN_CHECK_TIMEOUT_MS',
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
    'Emm_V5_Synchronous_motion\(0x00\)',
    '(?s)static CommandResult parseServoCommand\(const char \*line,.*?channel < 2U \|\| channel > 4U.*?channel < 4U && angle > 270U.*?channel == 4U && angle > 360U',
    '(?s)strncmp\(command, "servo ", 6U\) == 0.*?processServoCommand\(command\)',
    '(?s)static void stopServoMotion\(void\).*?targetAngleMdeg\[index\] = currentAngleMdeg\[index\]',
    '(?s)if \(emergencyStop\).*?stopServoMotion\(\);.*?stopAllMotors\(\)',
    '(?s)strcmp\(command, "stop"\) == 0.*?stopServoMotion\(\);.*?stopAllMotors\(\)',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_TIM2, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOA, pinSource, GPIO_AF_TIM2\)',
    'TIM_ITConfig\(TIM2, TIM_IT_Update, ENABLE\)',
    '\{1U, 1U, 0U, 0U, 1U\}',
    '\{0U, 0U, 1U, 1U, 0U\}',
    '\{1U, 1U, 1U, 0U, 0U\}',
    '\{0U, 0U, 0U, 1U, 1U\}'
)

foreach ($pattern in $requiredMainPatterns) {
    if ($main -notmatch $pattern) {
        throw "main.c does not satisfy contract pattern: $pattern"
    }
}

$forbiddenMainPatterns = @(
    'USART1',
    'GPIO_Pin_9',
    'GPIO_Pin_10',
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
