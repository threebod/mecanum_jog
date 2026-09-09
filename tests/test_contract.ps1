$ErrorActionPreference = 'Stop'

$exampleRoot = Split-Path -Parent $PSScriptRoot
$mainPath = Join-Path $exampleRoot 'main.c'
$projectPath = Join-Path $exampleRoot 'MecanumJog.uvprojx'
$readmePath = Join-Path $exampleRoot 'README.md'

foreach ($path in @($mainPath, $projectPath, $readmePath)) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Missing required file: $path"
    }
}

$main = Get-Content -LiteralPath $mainPath -Raw
$project = Get-Content -LiteralPath $projectPath -Raw

$requiredMainPatterns = @(
    '#define COMMAND_BAUD_RATE\s+115200U',
    '#define MOTOR_TEST_SPEED\s+30U',
    '#define MOTOR_TEST_ACCEL\s+50U',
    '#define MOTOR_TEST_PULSES\s+160U',
    '#define TEST_MOTOR_ID\s+5U',
    'RCC_APB1PeriphClockCmd\(RCC_APB1Periph_UART5, ENABLE\)',
    'GPIO_PinAFConfig\(GPIOC, GPIO_PinSource12, GPIO_AF_UART5\)',
    'GPIO_PinAFConfig\(GPIOD, GPIO_PinSource2, GPIO_AF_UART5\)',
    'void UART5_IRQHandler\(void\)',
    "received == '!'",
    'else if \(!emergencyStop &&',
    'if \(emergencyStop\)\s*\{\s*stopAllMotors\(\);\s*__disable_irq\(\);\s*emergencyStop = 0U;',
    'static void setAllMotorsEnabled\(bool enabled\)',
    'Emm_V5_En_Control\(id, enabled, false\)',
    '(?s)strcmp\(command, "enable"\) == 0.*?if \(!armed\).*?armed = 0U;.*?setAllMotorsEnabled\(true\)',
    '(?s)strcmp\(command, "disable"\) == 0.*?setAllMotorsEnabled\(false\).*?armed = 0U;',
    '(?s)strcmp\(command, "enable5"\) == 0.*?if \(!armed\).*?armed = 0U;.*?Emm_V5_En_Control\(TEST_MOTOR_ID, true, false\)',
    '(?s)strcmp\(command, "motor5 0"\) == 0.*?startMotor5Jog\(0U\)',
    '(?s)strcmp\(command, "motor5 1"\) == 0.*?startMotor5Jog\(1U\)',
    '(?s)static void startMotor5Jog\(uint8_t direction\).*?if \(!armed\).*?Emm_V5_Pos_Control\(TEST_MOTOR_ID, direction,.*?false, false\)',
    '(?s)strcmp\(command, "disable5"\) == 0.*?Emm_V5_Stop_Now\(TEST_MOTOR_ID, false\).*?Emm_V5_En_Control\(TEST_MOTOR_ID, false, false\).*?armed = 0U;',
    'for \(id = MOTOR_MIN_ID; id <= TEST_MOTOR_ID; \+\+id\)',
    'Emm_V5_Synchronous_motion\(0x00\)',
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
    'GPIO_Pin_10'
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
