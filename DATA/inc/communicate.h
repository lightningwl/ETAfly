#ifndef COMMUNICATE_H
#define COMMUNICATE_H

#include "sys.h"

/*************************************************************************/
/***************************** ���Ͳ��� ***********************************/

void SendImuData(int16_t ax,  int16_t ay,  int16_t az, int16_t gx, int16_t gy, int16_t gz);
void SendEularAngle(int16_t roll, int16_t pitch, int16_t yaw);
void SendMagBaroTempeData(int16_t mx, int16_t my, int16_t mz, int32_t baro, int16_t tempe);
void SendHeight(int32_t alt_fu, int32_t alt_add);
void SendFourPwmDuty(uint16_t pwm[4]);


/*************************************************************************/
/***************************** ���ղ��� ***********************************/

#define COMM_HEAD						0xAA		// ͨ��Э��֡ͷ
#define COMM_D_ADDR					0x66		// Ŀ���ַ

#define RC_CMD_LEN					10				// ��ת2 + ����2 + ����2 + ƫ��2 + ����1 + ״̬1
#define RC_CMD_ID						0xF1			// ����֡ID���Զ��壩

struct RC_CMD_t
{
	uint8_t head;			// ֡ͷ
	uint8_t d_addr;		// Ŀ���ַ
	uint8_t id;				// ������
	uint8_t len;			// ���ݳ���
	uint8_t data[RC_CMD_LEN];	// ��������
	uint8_t sc;				// ��У��
	uint8_t ac;				// ����У��
};

static uint8_t CheckFrame(const uint8_t *buf);

void RcCmd_Buf2Frame(const uint8_t *buf, struct RC_CMD_t *frame);


#endif

