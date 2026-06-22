
#include "comm.h"
#include "swarm_protocol.h"
#include "carrier_ctrl.h"
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
        Carrier_SetVel(c.vx,c.vy,c.wz,c.mode);break;}
    case MSG_SUPPLY_CMD:{MsgSupplyCmd_t s;memcpy(&s,pl,sizeof(s));
        Carrier_SupplyAction(s.action,s.slot);break;}
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
        MsgOdom_t om;float vx,vy,wz;
        /* Carrier暂时上报零里程计，后续接编码器 */
        Carrier_GetOdom(&om.pos_x,&om.pos_y,&om.yaw);
        om.vx_act=om.vy_act=om.wz_act=0;
        len=proto_build(s_tx,MSG_ODOM,&om,sizeof(om));
    } else {
        MsgBattery_t bat;
        bat.voltage=Carrier_GetBatteryVoltage();
        bat.current=0.0f;  /* TODO: 接电流传感器 */
        bat.percent=(uint8_t)Carrier_GetBattery();
        bat.charging=0;
        len=proto_build(s_tx,MSG_BATTERY,&bat,sizeof(bat));
    }
    s_toggle^=1;
    BSP_UART3_SendDMA(s_tx,len);
}
