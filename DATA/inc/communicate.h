#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "sys.h"

/*************************************************************************/
/***************************** 发送部分 ***********************************/

void SendImuData(int16_t ax,  int16_t ay,  int16_t az, int16_t gx, int16_t gy, int16_t gz);
void SendEularAngle(int16_t roll, int16_t pitch, int16_t yaw);
void SendMagBaroTempeData(int16_t mx, int16_t my, int16_t mz, int32_t baro, int16_t tempe);
void SendHeight(int32_t alt_fu, int32_t alt_add);
void SendFourPwmDuty(uint16_t pwm[4]);


/*************************************************************************/
/***************************** 接收部分 ***********************************/

#define COMM_HEAD						0xAA		// 通信协议帧头
#define COMM_D_ADDR					0x66		// 目标地址

#define RC_CMD_LEN					10				// 滚转2 + 俯仰2 + 油门2 + 偏航2 + 按键1 + 状态1
#define RC_CMD_ID						0xF1			// 数据帧ID（自定义）

struct RC_CMD_t
{
	uint8_t head;			// 帧头
	uint8_t d_addr;		// 目标地址
	uint8_t id;				// 功能码
	uint8_t len;			// 数据长度
	uint8_t data[RC_CMD_LEN];	// 数据内容
	uint8_t sc;				// 和校验
	uint8_t ac;				// 附加校验
};

static uint8_t CheckFrame(const uint8_t *buf);

void RcCmd_Buf2Frame(const uint8_t *buf, struct RC_CMD_t *frame);


#endif

