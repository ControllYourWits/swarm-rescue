
#include "comm.h"
#include "swarm_protocol.h"
#include "spec_ctrl.h"
#include "arm.h"
#include "led.h"
#include "bsp_uart.h"
#include <string.h>

static uint8_t s_tx[PROTO_MAX_PAYLOAD+8];
static uint8_t s_toggle=0;

typedef enum{PS_HEADER,PS_LEN,PS_DATA,PS_CRC,PS_TAIL}PS_t;
static PS_t s_ps=PS_HEADER;
static uint8_t s_buf[PROTO_MAX_PAYLOAD+4];
static uint8_t s_len=0,s_pos=0;

static void Dispatch(uint8_t id,const uint8_t *pl){
    switch(id){
    case MSG_CMD_VEL:{MsgCmdVel_t c;memcpy(&c,pl,sizeof(c));
        Spec_SetChassis(c.vx,c.vy,c.wz,c.mode);break;}
    case MSG_ARM_CMD:{MsgArmCmd_t a;memcpy(&a,pl,sizeof(a));
        Arm_SetAll(a.joint,a.gripper);break;}
    case MSG_LED_CMD:{MsgLedCmd_t l;memcpy(&l,pl,sizeof(l));
        LED_SetMode(l.mode,l.brightness,l.r,l.g,l.b);break;}
    default:break;}}

void BSP_UART3_RxCB(uint8_t b){
    switch(s_ps){
    case PS_HEADER:if(b==PROTO_HEADER)s_ps=PS_LEN;break;
    case PS_LEN:
        if(b>PROTO_MAX_PAYLOAD){s_ps=PS_HEADER;break;}
        s_len=b;s_buf[0]=b;s_pos=1;s_ps=PS_DATA;break;
    case PS_DATA:
        if(s_pos>=sizeof(s_buf)){s_ps=PS_HEADER;break;}
        s_buf[s_pos++]=b;if(s_pos==(uint8_t)(s_len+1))s_ps=PS_CRC;break;
    case PS_CRC:s_ps=(proto_crc8(s_buf,(uint16_t)(s_len+1))==b)?PS_TAIL:PS_HEADER;break;
    case PS_TAIL:if(b==PROTO_TAIL)Dispatch(s_buf[1],&s_buf[2]);s_ps=PS_HEADER;break;}}

void Comm_Init(void){BSP_UART3_Init();}

void Comm_TxTask(void){
    uint16_t len;
    if(s_toggle==0){
        MsgOdom_t om;Spec_GetOdom(&om.pos_x,&om.pos_y,&om.yaw);
        om.vx_act=om.vy_act=om.wz_act=0;
        len=proto_build(s_tx,MSG_ODOM,&om,sizeof(om));
    } else {
        MsgStatus_t st;st.robot_id=ROBOT_SPECIALIST;st.mode=0;
        st.motor_ok=0xFF;st.error_code=0;st.battery_v=22.0f;
        len=proto_build(s_tx,MSG_STATUS,&st,sizeof(st));
    }
    s_toggle^=1;BSP_UART3_SendDMA(s_tx,len);}
