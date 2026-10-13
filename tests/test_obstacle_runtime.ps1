$ErrorActionPreference = 'Stop'
$env:Path = "E:\Qt\Tools\mingw1310_64\bin;$env:Path"
$root = Split-Path -Parent $PSScriptRoot
$source = Get-Content (Join-Path $root 'main.c') -Raw
$irq = $source.Substring($source.IndexOf('void USART3_IRQHandler(void)'))
$irq = $irq.Substring(0, $irq.IndexOf('static void serviceQrScan('))
$loop = $source.Substring($source.IndexOf('        if (emergencyStop) {', $source.IndexOf('    for (;;) {')))
$loop = $loop.Substring(0, $loop.IndexOf('        if (rxReady) {'))
$prefix = @'
#include <assert.h>
#include <string.h>
#include "../obstacle_stop.h"
#include "../qr_scan.h"
#include "../avoidance_run.h"
#define USART3 3
#define USART_IT_RXNE 1
#define RESET 0
static QrScanParser qrParser;
static ObstacleStopParser obstacleParser;
static AvRun avoidance;
static AvParser avoidanceParser;
static volatile AvObservation avoidanceRxPacket;
static volatile uint8_t avoidanceRxReady;
static volatile unsigned char obstacleStopPending, emergencyStop, qrRxReady;
static char qrRxPayload[QR_SCAN_PAYLOAD_MAX + 1U];
static unsigned char inputByte, navInitialized, fullRouteRunning, missionButtonReady;
static unsigned char motionMode, rxLength, rxReady, armed;
static unsigned stops, servoStops;
static char output[1024];
static int USART_GetITStatus(int port, int flag) { (void)port; (void)flag; return 1; }
static unsigned USART_ReceiveData(int port) { (void)port; return inputByte; }
static void USART_ClearITPendingBit(int port, int flag) { (void)port; (void)flag; }
static int visionSessionActive(void) { return 0; }
static void stopServoMotion(void) { ++servoStops; }
static void stopAllMotors(void) { ++stops; armed = navInitialized = fullRouteRunning = 0; }
static void __disable_irq(void) {}
static void __enable_irq(void) {}
static void failVision(const char *reason) { (void)reason; }
static void serialSendString(const char *line) { strcat(output, line); }
'@
$suffix = @'
static void receive(const char *line) {
    while (*line) { inputByte = (unsigned char)*line++; USART3_IRQHandler(); }
}
static void cameraResult(unsigned token,unsigned region,unsigned pixels,int corrupt) {
    char line[96];unsigned size=(unsigned)sprintf(line,"AVR:%u:%u:1:1:320:407:%u",token,region,pixels);
    sprintf(line+size,":%04X\n",(unsigned)(av_crc16(line,size)^(corrupt?1U:0U)));
    receive(line);
}
int main(void) {
    avoidance.active=1;avoidance.token=17;avoidance.region=3;
    cameraResult(16,3,2300,0);cameraResult(17,4,2300,0);
    cameraResult(17,3,2199,0);cameraResult(17,3,2300,1);
    assert(!avoidanceRxReady && !avoidance.blocked && !emergencyStop);
    cameraResult(17,3,2300,0);
    assert(avoidanceRxReady && avoidanceRxPacket.token==17 && avoidanceRxPacket.stable && !avoidance.blocked);
    avoidance.active=0;avoidanceRxReady=0;
    receive("QR:OB:STOP\n");
    assert(qrRxReady && !emergencyStop);
    qrRxReady = 0;
    receive("OB:STOPX\nOB:ST");
    assert(!emergencyStop);
    receive("OP\n");
    assert(emergencyStop && obstacleStopPending && !qrRxReady);
    armed = navInitialized = fullRouteRunning = 1;
    rxReady = rxLength = 1;
    serviceStop();
    assert(stops == 1 && servoStops == 1 && !armed && !navInitialized && !fullRouteRunning);
    assert(!rxReady && !rxLength && !emergencyStop && !obstacleStopPending);
    assert(strstr(output, "OBSTACLE STOP;") != 0);
    receive("QR:156+123+516+231\n");
    assert(qrRxReady && !emergencyStop);
    assert(strcmp(qrRxPayload, "156+123+516+231") == 0);
    return 0;
}
'@
$testSource = Join-Path $PSScriptRoot '_obstacle_runtime.c'
$testExe = Join-Path $PSScriptRoot '_obstacle_runtime.exe'
try {
    [IO.File]::WriteAllText($testSource, "$prefix`n$irq`nstatic void serviceStop(void) {`n$loop`n}`n$suffix")
    & gcc -std=c99 -Wall -Wextra $testSource -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Obstacle runtime compilation failed' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Obstacle runtime assertions failed' }
    Write-Output 'PASS: actual USART3 receiver and emergency stop service'
} finally {
    Remove-Item -LiteralPath $testSource, $testExe -ErrorAction SilentlyContinue
}
