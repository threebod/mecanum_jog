#include "platform.h"
#include "stm32f4xx.h"
#include "Emm_V5.h"
#include <math.h>
#include <string.h>
static volatile uint32_t ticks;
static volatile uint8_t fault,overflow;
static volatile uint16_t wr,rd;
static uint8_t camera_rx[256];
static uint8_t console_rx[256];
static volatile uint16_t console_wr,console_rd;
static char console[128];static unsigned console_n;static int discard_line;
static int referenced;static uint32_t wait_until;
static float pos[5];
static float servo_target[3],servo_start[3];static uint32_t servo_at,servo_duration;
static int moving_servo=-1;
uint32_t platform_ms(void) {return ticks;}
void SysTick_Handler(void) {ticks++;}
void HardFault_Handler(void) { /* No further motion commands. Reset and re-reference. */
    for(;;) {}
}
static void gpio_af(GPIO_TypeDef *port,uint16_t pins,uint8_t a,uint8_t b,uint8_t af) {
    GPIO_InitTypeDef g;GPIO_StructInit(&g);g.GPIO_Pin=pins;g.GPIO_Mode=GPIO_Mode_AF;
    g.GPIO_Speed=GPIO_Speed_50MHz;g.GPIO_OType=GPIO_OType_PP;g.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_Init(port,&g);GPIO_PinAFConfig(port,a,af);GPIO_PinAFConfig(port,b,af);
}
static void uart_init(USART_TypeDef *u) {
    USART_InitTypeDef s;USART_StructInit(&s);s.USART_BaudRate=115200;
    s.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;USART_Init(u,&s);USART_Cmd(u,ENABLE);
}
void platform_init(void) {
    CAN_InitTypeDef c;CAN_FilterInitTypeDef f;TIM_TimeBaseInitTypeDef t;TIM_OCInitTypeDef o;
    GPIO_InitTypeDef g;RCC_ClocksTypeDef clocks;
    SystemCoreClockUpdate();SysTick_Config(SystemCoreClock/1000);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4|RCC_APB1Periph_CAN1|RCC_APB1Periph_TIM2,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    gpio_af(GPIOC,GPIO_Pin_10|GPIO_Pin_11,10,11,GPIO_AF_UART4);
    gpio_af(GPIOA,GPIO_Pin_9|GPIO_Pin_10,9,10,GPIO_AF_USART1);
    uart_init(UART4);uart_init(USART1);
    USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);NVIC_EnableIRQ(UART4_IRQn);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);NVIC_EnableIRQ(USART1_IRQn);
    gpio_af(GPIOA,GPIO_Pin_11|GPIO_Pin_12,11,12,GPIO_AF_CAN1);
    CAN_StructInit(&c);c.CAN_ABOM=ENABLE;c.CAN_NART=ENABLE;c.CAN_TXFP=ENABLE;
    c.CAN_Prescaler=7;c.CAN_BS1=CAN_BS1_9tq;c.CAN_BS2=CAN_BS2_2tq;
    if(CAN_Init(CAN1,&c)!=CAN_InitStatus_Success) fault=1;
    memset(&f,0,sizeof(f));f.CAN_FilterMode=CAN_FilterMode_IdMask;f.CAN_FilterScale=CAN_FilterScale_32bit;
    f.CAN_FilterFIFOAssignment=CAN_FIFO0;f.CAN_FilterActivation=ENABLE;CAN_FilterInit(&f);
    gpio_af(GPIOA,GPIO_Pin_1|GPIO_Pin_2,1,2,GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA,3,GPIO_AF_TIM2);GPIO_StructInit(&g);
    g.GPIO_Pin=GPIO_Pin_3;g.GPIO_Mode=GPIO_Mode_AF;g.GPIO_OType=GPIO_OType_PP;GPIO_Init(GPIOA,&g);
    RCC_GetClocksFreq(&clocks);
    TIM_TimeBaseStructInit(&t);t.TIM_Prescaler=(uint16_t)((clocks.PCLK1_Frequency*(clocks.PCLK1_Frequency==clocks.HCLK_Frequency?1:2))/1000000-1);
    t.TIM_Period=19999;TIM_TimeBaseInit(TIM2,&t);
    TIM_OCStructInit(&o);o.TIM_OCMode=TIM_OCMode_PWM1;o.TIM_OutputState=TIM_OutputState_Enable;o.TIM_Pulse=0;
    TIM_OC2Init(TIM2,&o);TIM_OC3Init(TIM2,&o);TIM_OC4Init(TIM2,&o);TIM_Cmd(TIM2,ENABLE);
    /* Zero-width pulses until explicit manual reference confirmation. */
    GPIO_StructInit(&g);g.GPIO_Pin=GPIO_Pin_9;g.GPIO_Mode=GPIO_Mode_IN;g.GPIO_PuPd=GPIO_PuPd_UP;GPIO_Init(GPIOC,&g);
    RCC_GetClocksFreq(&clocks);if(clocks.SYSCLK_Frequency!=168000000) fault=1;
}
void UART4_IRQHandler(void) {
    uint32_t sr=UART4->SR;uint8_t b=(uint8_t)UART4->DR;uint16_t next;
    if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {overflow=1;return;}
    if(sr&USART_SR_RXNE) {next=(uint16_t)((wr+1)&255);if(next==rd) overflow=1;else {camera_rx[wr]=b;wr=next;}}
}
int platform_camera_byte(uint8_t *b) {if(rd==wr)return 0;*b=camera_rx[rd];rd=(uint16_t)((rd+1)&255);return 1;}
void USART1_IRQHandler(void) {
    uint32_t sr=USART1->SR;uint8_t b=(uint8_t)USART1->DR;uint16_t next;
    if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE)) {overflow=1;return;}
    if(sr&USART_SR_RXNE) {next=(uint16_t)((console_wr+1)&255);if(next==console_rd)overflow=1;
        else {console_rx[console_wr]=b;console_wr=next;}}
}
static int send_byte(USART_TypeDef *u,uint8_t b) {
    uint32_t start=ticks;
    while(USART_GetFlagStatus(u,USART_FLAG_TXE)==RESET) if(ticks-start>5) {fault=1;return 0;}
    USART_SendData(u,b);return 1;
}
void platform_camera_send(const Packet *p) {uint8_t data[PACKET_SIZE];unsigned i;protocol_encode(p,data);for(i=0;i<PACKET_SIZE;i++) if(!send_byte(UART4,data[i]))break;}
void platform_log(const char *s) {while(*s)if(!send_byte(USART1,(uint8_t)*s++))return;send_byte(USART1,'\r');send_byte(USART1,'\n');}
int platform_line(char *dst,unsigned cap) {
    char b;
    while(console_rd!=console_wr) {
        b=(char)console_rx[console_rd];console_rd=(uint16_t)((console_rd+1)&255);
        if(b=='\n'||b=='\r') {
            if(discard_line) {discard_line=0;console_n=0;continue;}
            if(console_n) {console[console_n]=0;if(console_n>=cap)console_n=cap-1;memcpy(dst,console,console_n);dst[console_n]=0;console_n=0;return 1;}
        } else if(!discard_line) {if(console_n<sizeof(console)-1)console[console_n++]=b;else {discard_line=1;console_n=0;}}
    }return 0;
}
/* Same Emm CAN fragmentation as the original driver, with bounded TX confirmation. */
void can_SendCmd(volatile uint8_t *cmd,uint8_t len) {
    unsigned i=2,pack=0,k;CanTxMsg msg;uint8_t mailbox,status;uint32_t start;
    if(len<3|| (cmd[0]!=5&&cmd[0]!=6)) {fault=1;return;}
    while(i<len) {
        memset(&msg,0,sizeof(msg));msg.ExtId=((uint32_t)cmd[0]<<8)|pack++;
        msg.IDE=CAN_Id_Extended;msg.RTR=CAN_RTR_Data;msg.Data[0]=cmd[1];
        for(k=1;k<8&&i<len;k++)msg.Data[k]=cmd[i++];
        msg.DLC=(uint8_t)k;
        mailbox=CAN_Transmit(CAN1,&msg);if(mailbox==CAN_TxStatus_NoMailBox){fault=1;return;}
        start=ticks;
        do {status=CAN_TransmitStatus(CAN1,mailbox);if(ticks-start>20) {CAN_CancelTransmit(CAN1,mailbox);fault=1;return;}} while(status==CAN_TxStatus_Pending);
        if(status!=CAN_TxStatus_Ok){fault=1;return;}
    }
}
static void pwm(unsigned n,float degrees) {
    uint16_t pulse=(uint16_t)(500+2000*degrees/config.servo_span[n]);
    if(n==0)TIM_SetCompare2(TIM2,pulse);else if(n==1)TIM_SetCompare3(TIM2,pulse);else TIM_SetCompare4(TIM2,pulse);
}
int platform_busy(void) {return (int32_t)(wait_until-ticks)>0;}
int platform_fault(void) {return fault||overflow;}
int platform_referenced(void) {return referenced;}
float platform_position(unsigned c) {return c<5?pos[c]:0;}
int platform_zero(float x,float z,float theta,float tray,float grip) {
    unsigned i;float q[5];
    if(platform_busy()||platform_fault())return 0;
    q[0]=x;q[1]=z;q[2]=grip;q[3]=tray;q[4]=theta;
    for(i=0;i<5;i++)if(!isfinite(q[i]))return 0;
    for(i=0;i<2;i++)if(q[i]<config.axis_min[i]||q[i]>config.axis_max[i])return 0;
    for(i=0;i<3;i++)if(q[i+2]<config.servo_min[i]||q[i+2]>config.servo_max[i])return 0;
    /* Only software reference for steppers, never trigger motor homing. */
    memcpy(pos,q,sizeof(pos));for(i=0;i<3;i++) {servo_target[i]=q[i+2];pwm(i,q[i+2]);}
    Emm_V5_En_Control(5,true,false);Emm_V5_En_Control(6,true,false);
    if(platform_fault())return 0;
    referenced=1;wait_until=ticks+config.settle_ms;return 1;
}
int platform_axis(unsigned a,float target) {
    float delta;uint32_t steps;uint8_t dir;
    if(a>1||!referenced||platform_busy()||platform_fault()||!isfinite(target)||
       target<config.axis_min[a]||target>config.axis_max[a])return 0;
    delta=target-pos[a];steps=(uint32_t)(fabsf(delta)*config.steps_mm[a]+0.5f);
    dir=delta>=0?config.positive_dir[a]:(uint8_t)!config.positive_dir[a];
    if(steps)Emm_V5_Pos_Control(a?5:6,dir,config.motor_rpm,config.motor_acc,steps,false,false);
    if(platform_fault())return 0;
    pos[a]+= (delta>=0?1:-1)*(float)steps/config.steps_mm[a];
    wait_until=ticks+(uint32_t)(fabsf(delta)*config.ms_mm[a])+config.settle_ms;return 1;
}
int platform_servo(unsigned s,float degrees) {
    if(s>2||!referenced||platform_busy()||platform_fault()||!isfinite(degrees)||
       degrees<config.servo_min[s]||degrees>config.servo_max[s])return 0;
    servo_start[s]=pos[s+2];servo_target[s]=degrees;servo_at=ticks;
    servo_duration=(uint32_t)(fabsf(degrees-pos[s+2])*config.servo_ms_degree)+100;
    moving_servo=(int)s;wait_until=ticks+servo_duration+config.settle_ms;return 1;
}
void platform_poll(void) {
    CanRxMsg rx;
    if(moving_servo>=0) {
        unsigned s=(unsigned)moving_servo;float t=(float)(ticks-servo_at)/servo_duration;
        if(t>=1) {t=1;moving_servo=-1;}
        pos[s+2]=servo_start[s]+(servo_target[s]-servo_start[s])*(t*t*(3-2*t));pwm(s,pos[s+2]);
    }
    while(CAN_MessagePending(CAN1,CAN_FIFO0)) CAN_Receive(CAN1,CAN_FIFO0,&rx);
    if(CAN_GetFlagStatus(CAN1,CAN_FLAG_BOF)) fault=1;
}
void platform_stop(void) {
    /* Hold the last commanded servo position; do not open a loaded gripper. */
    moving_servo=-1;referenced=0;wait_until=ticks;
    Emm_V5_Stop_Now(5,false);Emm_V5_Stop_Now(6,false);
}
int platform_stop_pressed(void) {
    static uint32_t low_at;static int low;
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)==Bit_RESET) {
        if(!low){low=1;low_at=ticks;}return ticks-low_at>=30;
    }low=0;return 0;
}
