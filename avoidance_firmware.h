/* Included by main.c after existing motor, IMU and command helpers. */
#ifndef MECANUM_AVOIDANCE_FIRMWARE_H
#define MECANUM_AVOIDANCE_FIRMWARE_H

static uint32_t avoidanceTick,avoidanceLastRequest,avoidanceLastReport,avoidanceTurnStable;
static uint16_t avoidanceLastToken,avoidanceSnapshotGeneration;
static int16_t avoidanceMapUp,avoidanceMapRight;
static AvPhase avoidancePreviousPhase;
static unsigned avoidancePreviousIndex,avoidanceSnapshotIndex,avoidanceSnapshotCount;
static AvPoint avoidanceSnapshot[AV_PATH_MAX];

static void reportAvoidanceObservation(const AvObservation *packet)
{
    serialSendString("AV OBS token=");serialSendUint(packet->token);
    serialSendString(" region=");serialSendUint(packet->region);
    serialSendString(" u=");serialSendUint(packet->u);
    serialSendString(" bottom=");serialSendUint(packet->bottom);
    serialSendString(" pixels=");serialSendUint32(packet->pixels);
    serialSendString(" gap=");serialSendInt((int16_t)(479-packet->bottom));
    serialSendString(" front_mm=");serialSendInt((int16_t)av_front_distance(packet->bottom));
    serialSendString("\r\n");
}

static void reportAvoidance(void)
{
    serialSendString("AV STATE state=");serialSendString(av_phase_name(avoidance.phase));
    serialSendString(" active=");serialSendUint(avoidance.active);
    serialSendString(" round=");serialSendUint(avoidance.stage<4?1:2);
    serialSendString(" target=");serialSendString(av_target_name(avoidance.stage));
    serialSendString(" region=");serialSendUint(avoidance.region);
    serialSendString(" blocked=");serialSendUint(avoidance.blocked);
    serialSendString(" replans=");serialSendUint(avoidance.replans);
    serialSendString(" camera=");serialSendUint(avoidance.camera);
    serialSendString(" x=");serialSendInt((int16_t)(avoidance.x+0.5f));
    serialSendString(" y=");serialSendInt((int16_t)(avoidance.y+0.5f));
    serialSendString(" yaw_cdeg=");serialSendInt(navFieldYawCdeg());
    serialSendString(" reason=");serialSendString(avoidance.reason?avoidance.reason:"NONE");
    serialSendString("\r\n");avoidance.dirty=0;avoidanceLastReport=clockMs;
}
static void beginAvoidanceSnapshot(void)
{
    unsigned i;avoidanceSnapshotGeneration=avoidance.generation;
    avoidanceSnapshotCount=avoidance.count;avoidanceSnapshotIndex=0;
    for(i=0;i<avoidance.count;++i)avoidanceSnapshot[i]=avoidance.path[i];
    serialSendString("AV PATH_BEGIN version=");serialSendUint(avoidanceSnapshotGeneration);
    serialSendString(" count=");serialSendUint((uint16_t)avoidanceSnapshotCount);serialSendString("\r\n");
}
static void serviceAvoidanceSnapshot(void)
{
    if(avoidanceSnapshotIndex<avoidanceSnapshotCount) {
        AvPoint p=avoidanceSnapshot[avoidanceSnapshotIndex];
        serialSendString("AV PATH_POINT version=");serialSendUint(avoidanceSnapshotGeneration);
        serialSendString(" index=");serialSendUint((uint16_t)avoidanceSnapshotIndex++);
        serialSendString(" x=");serialSendInt(p.x);serialSendString(" y=");serialSendInt(p.y);serialSendString("\r\n");
    } else if(avoidanceSnapshotIndex==avoidanceSnapshotCount) {
        serialSendString("AV PATH_END version=");serialSendUint(avoidanceSnapshotGeneration);serialSendString("\r\n");
        ++avoidanceSnapshotIndex;
    }
}
static uint8_t processAvoidanceCommand(const char *command)
{
    uint16_t zone,rpm;
    if(strcmp(command,"avoid status")==0) {reportAvoidance();beginAvoidanceSnapshot();return 1;}
    if(strcmp(command,"avoid stop")==0) {
        av_run_stop(&avoidance,"STOPPED");stopServoMotion();stopAllMotors();
        avoidanceMapUp=avoidanceMapRight=0;reportAvoidance();return 1;
    }
    if(strcmp(command,"avoid clear")==0) {
        if(avoidance.active || routeActive || motionMode || mechanismActionActive || visionSessionActive() ||
            servoChannelMoving[0] || servoChannelMoving[1] || servoChannelMoving[2]) {
            serialSendString("ERR AV: clear requires stopped vehicle\r\n");return 1;
        }
        avoidance.blocked=0;avoidance.count=0;++avoidance.generation;
        avoidance.reason="CLEARED";reportAvoidance();beginAvoidanceSnapshot();return 1;
    }
    if(strncmp(command,"avoid start ",12)!=0) {
        if(strncmp(command,"avoid ",6)==0) {serialSendString("ERR AV: use avoid start Z RPM|stop|status|clear\r\n");return 1;}
        return 0;
    }
    if(!parsePair(command+12,&zone,&rpm) || (zone!=1 && zone!=2) || rpm<10 || rpm>250) {
        serialSendString("ERR AV: start requires zone 1|2 and 10..250 RPM\r\n");return 1;
    }
    if(!armed || routeActive || motionMode || mechanismActionActive || visionSessionActive() || avoidance.active ||
        servoChannelMoving[0] || servoChannelMoving[1] || servoChannelMoving[2] ||
        !imuValid || clockMs-imuStamp>250 || motionInterrupted()) {
        serialSendString("ERR AV: arm, idle, fresh IMU and healthy CAN required\r\n");return 1;
    }
    armed=0;
    if(!av_run_start(&avoidance,(uint8_t)zone,rpm,clockMs)) {serialSendString("ERR AV: blocked start\r\n");return 1;}
    qrResultAtStation=0;
    routeActive=routeWaiting=1U;routeAuto=0U;navRunning=navInitialized=fullRouteRunning=0;
    routeX=avoidance.x;routeY=avoidance.y;routeBaseYaw=routeYaw=imuYaw;routeHeading=0;
    routeRpm=rpm;routeSentValid=0;routeDriveRpm=routeTurnRpm=routeCorrection=0;
    routeHeadingPidReset(&routeHeadingPid);routeTurnPidReset(&routeTurnPid);
    avoidanceTick=clockMs;avoidanceLastToken=0;avoidanceMapUp=avoidanceMapRight=0;
    avoidancePreviousPhase=AV_IDLE;avoidancePreviousIndex=0;
    __disable_irq();avoidanceRxReady=0;__enable_irq();
    setAllMotorsEnabled(true);
    if(motionInterrupted()) {av_fail(&avoidance,"CAN_FAULT");stopAllMotors();}
    reportAvoidance();return 1;
}
static void serviceAvoidance(void)
{
    uint32_t now=clockMs,dt=now-avoidanceTick;AvObservation packet;AvPoint target;
    float error,remaining;int16_t bodyForward,bodyRight,turn;uint8_t lateral,reportObservation=0;
    if(avoidance.active && !routeActive) {
        av_run_stop(&avoidance,"CANCELLED");avoidanceMapUp=avoidanceMapRight=0;reportAvoidance();
    }
    if(!avoidance.active) {serviceAvoidanceSnapshot();return;}
    if(dt<20) return;
    avoidanceTick=now;
    if(!imuValid || now-imuStamp>250 || dt>150 || motionInterrupted()) av_fail(&avoidance,"CONTROL_FAULT");
    if(avoidance.active) {
        lateral=(uint8_t)(avoidance.heading==1 || avoidance.heading==3);
        avoidance.x+=routeEstimateScaled(avoidanceMapRight,dt,lateral?routeForwardScale:routeLateralScale);
        avoidance.y+=routeEstimateScaled(avoidanceMapUp,dt,lateral?routeLateralScale:routeForwardScale);
        if(avoidanceRxReady) {
            __disable_irq();packet=avoidanceRxPacket;avoidanceRxReady=0;__enable_irq();
            reportObservation=(uint8_t)(packet.token==avoidance.token && packet.region==avoidance.region && packet.seen &&
                (avoidance.phase==AV_MOVE || (avoidance.phase==AV_CONFIRM && packet.stable)));
            av_run_observe(&avoidance,&packet,now);
        }
        if(avoidance.phase==AV_DWELL && avoidance.stage==0 && !qrResultAtStation) {
            avoidance.deadline=now+500;
            if(strcmp(avoidance.reason,"QR_WAIT")!=0) {avoidance.reason="QR_WAIT";avoidance.dirty=1;}
        } else if(avoidance.phase==AV_DWELL && avoidance.stage==0) avoidance.reason="NONE";
        av_run_tick(&avoidance,now);
    }
    if(!avoidance.active) {
        stopAllMotors();avoidanceMapUp=avoidanceMapRight=0;
        if(reportObservation)reportAvoidanceObservation(&packet);
        reportAvoidance();beginAvoidanceSnapshot();return;
    }
    /* Moving observations get a fresh request id; stationary confirmation keeps
       one id so the camera can accumulate three consecutive frames. */
    if(avoidanceLastToken==avoidance.token && now-avoidanceLastRequest>=200 &&
        avoidance.phase!=AV_CAMERA && avoidance.phase!=AV_CONFIRM)
        av_new_request(&avoidance,avoidance.region,avoidance.requestConfirm,now);
    if(avoidanceLastToken!=avoidance.token || now-avoidanceLastRequest>=200) {
        char line[64];unsigned i,n=av_encode_request(line,avoidance.token,avoidance.region,avoidance.requestConfirm);
        for(i=0;i<n;++i) {
            while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET) {}
            USART_SendData(USART3,(uint8_t)line[i]);
        }
        avoidanceLastToken=avoidance.token;avoidanceLastRequest=clockMs;
    }
    if(avoidance.phase!=avoidancePreviousPhase || avoidance.index!=avoidancePreviousIndex) {
        stopDriveMotors();routeSentValid=0;avoidanceMapUp=avoidanceMapRight=0;
        routeDriveRpm=routeTurnRpm=routeCorrection=0;
        routeHeadingPidReset(&routeHeadingPid);routeTurnPidReset(&routeTurnPid);avoidanceTurnStable=0;
        avoidancePreviousPhase=avoidance.phase;avoidancePreviousIndex=avoidance.index;
    }
    if(reportObservation)reportAvoidanceObservation(&packet);
    routeX=avoidance.x;routeY=avoidance.y;routeHeading=avoidance.heading==3?-1:(int8_t)avoidance.heading;
    if(avoidance.phase==AV_TURN || avoidance.phase==AV_MOVE || avoidance.phase==AV_RETREAT) {
        target=av_leg_target(&avoidance);
        if(now-avoidance.legStarted>(avoidance.phase==AV_TURN?18000U:45000U)) av_fail(&avoidance,"LEG_TIMEOUT");
        if(!avoidance.active) {stopAllMotors();avoidanceMapUp=avoidanceMapRight=0;reportAvoidance();return;}
        routeYaw=routeBaseYaw+(avoidance.phase==AV_TURN?(target.heading==3?-1:target.heading):routeHeading)*90.0f*yawSign;
        error=headingError(routeYaw,imuYaw);
        if(avoidance.phase==AV_TURN) {
            turn=routeRound(routeTurnRamp(routeTurnRpm,routeAbs(error)<=2?0:
                routeTurnPidStep(&routeTurnPid,error,imuStamp,routeTurnRpmLimit,yawSign),dt));
            routeTurnRpm=(float)turn;sendRouteSpeeds(0,0,turn);
            if(routeAbs(error)<=2 && turn==0) {
                if(!avoidanceTurnStable) avoidanceTurnStable=now;
                if(now-avoidanceTurnStable>=200) av_run_arrived(&avoidance,now);
            } else avoidanceTurnStable=0;
        } else if(avoidance.active) {
            float dx=target.x-avoidance.x,dy=target.y-avoidance.y;
            if((avoidanceMapRight>0 && dx<0)||(avoidanceMapRight<0 && dx>0)) {avoidance.x=target.x;dx=0;}
            if((avoidanceMapUp>0 && dy<0)||(avoidanceMapUp<0 && dy>0)) {avoidance.y=target.y;dy=0;}
            remaining=routeAbs(dx)+routeAbs(dy);
            if(routeAbs(error)>20) av_fail(&avoidance,"HEADING_FAULT");
            else if(remaining<=1.5f) {
                stopDriveMotors();avoidanceMapUp=avoidanceMapRight=0;routeSentValid=0;av_run_arrived(&avoidance,now);
            } else {
                uint16_t cruise=avoidance.rpm;
                routeBody(dy?1:0,dx?1:0,routeHeading,&bodyForward,&bodyRight);
                if(bodyRight && cruise>routeLateralRpmLimit)cruise=routeLateralRpmLimit;
                /* Braking envelope and acceleration match the existing chassis controller. */
                routeDriveRpm=routeSlew(routeDriveRpm,(float)routeSpeed(remaining,now-avoidance.legStarted,cruise),ROUTE_ACCEL_RPM_S,dt);
                avoidanceMapUp=dy?(int16_t)((dy>0?1:-1)*routeRound(routeDriveRpm)):0;
                avoidanceMapRight=dx?(int16_t)((dx>0?1:-1)*routeRound(routeDriveRpm)):0;
                routeBody(avoidanceMapUp,avoidanceMapRight,routeHeading,&bodyForward,&bodyRight);
                turn=routeRound(routeHeadingPidStep(&routeHeadingPid,&routeHeadingGains,error,imuYaw,imuStamp,dt,yawSign));
                sendRouteSpeeds(bodyForward,bodyRight,turn);
            }
        }
    } else {avoidanceMapUp=avoidanceMapRight=0;sendRouteSpeeds(0,0,0);}
    if(!avoidance.active) {stopAllMotors();avoidanceMapUp=avoidanceMapRight=0;}
    if(avoidanceSnapshotGeneration!=avoidance.generation)beginAvoidanceSnapshot();
    if(avoidance.dirty || now-avoidanceLastReport>=200)reportAvoidance();
    serviceAvoidanceSnapshot();
}
#endif
