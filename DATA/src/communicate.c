#include "communicate.h"
#include "debug.h"
#include "uavmath.h"


/*************************************************************************/
/***************************** ���Ͳ��� ***********************************/

/*
 * ��IMU���ݷ��͵���λ��
 * ͨ��Э�飺����ͨ��Э��
 */
void SendImuData(int16_t ax,  int16_t ay,  int16_t az, int16_t gx, int16_t gy, int16_t gz)
{
	uint8_t cnt=0;
	uint8_t i;
	uint8_t sc=0;
	uint8_t ac=0;
	
	uint8_t data[32] = {0};
	
	data[cnt++] = 0xAA;							// ֡ͷ 0xAA
	data[cnt++] = 0xFF;							// Ŀ���ַ 0xFF
	data[cnt++] = 0x01;							// ������
	data[cnt++] = 13;								// ���ݳ���
	
	data[cnt++] = (uint8_t) ax;			// ȡ��8λ
	data[cnt++] = ax >> 8;					// ȡ��8λ
	
	data[cnt++] = (uint8_t) ay;
	data[cnt++] = ay >> 8;
	
	data[cnt++] = (uint8_t) az;
	data[cnt++] = az >> 8;
	
	data[cnt++] = (uint8_t) gx;
	data[cnt++] = gx >> 8;
	
	data[cnt++] = (uint8_t) gy;
	data[cnt++] = gy >> 8;
	
	data[cnt++] = (uint8_t) gz;
	data[cnt++] = gz >> 8;
	
	data[cnt++] = 1;


	for(i=0; i < (data[3]+4); i++)
	{
	 sc += data[i]; 									// CRCУ��1
	 ac += sc; 												// CRCУ��2
	}
	
	data[cnt++]=sc;
	data[cnt++]=ac;

	PrintDataApi(data, cnt);
}

/*
 * ��ŷ�������ݷ��͵���λ��
 * ͨ��Э�飺����ͨ��Э��
 */
void SendEularAngle(int16_t roll, int16_t pitch, int16_t yaw)
{
	uint8_t cnt=0;
	uint8_t i;
	uint8_t sc=0;
	uint8_t ac=0;
	
	uint8_t data[32] = {0};
	
	data[cnt++] = 0xAA;							// ֡ͷ 0xAA
	data[cnt++] = 0xFF;							// Ŀ���ַ 0xFF
	data[cnt++] = 0x03;							// ������
	data[cnt++] = 7;								// ���ݳ���
	
	data[cnt++] = (uint8_t) roll;			// ȡ��8λ
	data[cnt++] = roll >> 8;					// ȡ��8λ
	
	data[cnt++] = (uint8_t)  pitch;
	data[cnt++] = pitch >> 8;
	
	data[cnt++] = (uint8_t) yaw;
	data[cnt++] = yaw >> 8;
	
	data[cnt++] = 1;


	for(i=0; i < (data[3]+4); i++)
	{
	 sc += data[i]; 									// CRCУ��1
	 ac += sc; 												// CRCУ��2
	}
	
	data[cnt++]=sc;
	data[cnt++]=ac;

	PrintDataApi(data, cnt);
}


/*
 * ������������ѹ�����ݷ��͵���λ��
 * ͨ��Э�飺����ͨ��Э��
 */
void SendMagBaroTempeData(int16_t mx, int16_t my, int16_t mz, int32_t baro, int16_t tempe)
{
	uint8_t cnt=0;
	uint8_t i;
	uint8_t sc=0;
	uint8_t ac=0;
	
	uint8_t data[32] = {0};
	
	data[cnt++] = 0xAA;							// ֡ͷ 0xAA
	data[cnt++] = 0xFF;							// Ŀ���ַ 0xFF
	data[cnt++] = 0x02;							// ������
	data[cnt++] = 14;								// ���ݳ���
	
	data[cnt++] = (uint8_t) mx;			// ȡ��8λ
	data[cnt++] = mx >> 8;					// ȡ��8λ
	
	data[cnt++] = (uint8_t) my;
	data[cnt++] = my >> 8;
	
	data[cnt++] = (uint8_t) mz;
	data[cnt++] = mz >> 8;

	data[cnt++] = (uint8_t) baro;		// [7...0]
	data[cnt++] = (uint8_t) (baro >> 8);	// [15...8]
	data[cnt++] = (uint8_t) (baro >> 16);	// [23...16]
	data[cnt++] = (uint8_t) (baro >> 24);	// [31...24]
	
	data[cnt++] = (uint8_t) tempe;
	data[cnt++] = tempe >> 8;

	data[cnt++] = 1;
	data[cnt++] = 1;

	for(i=0; i < (data[3]+4); i++)
	{
	 sc += data[i]; 									// CRCУ��1
	 ac += sc; 												// CRCУ��2
	}
	
	data[cnt++]=sc;
	data[cnt++]=ac;

	PrintDataApi(data, cnt);
}

/*
 * ���߶����ݷ��͵���λ��
 * ͨ��Э�飺����ͨ��Э��
 */
void SendHeight(int32_t alt_fu, int32_t alt_add)
{
	uint8_t cnt=0;
	uint8_t i;
	uint8_t sc=0;
	uint8_t ac=0;
	
	uint8_t data[32] = {0};
	
	data[cnt++] = 0xAA;							// ֡ͷ 0xAA
	data[cnt++] = 0xFF;							// Ŀ���ַ 0xFF
	data[cnt++] = 0x05;							// ������
	data[cnt++] = 9;								// ���ݳ���
	

	data[cnt++] = (uint8_t) alt_fu;		// [7...0]
	data[cnt++] = (uint8_t) (alt_fu >> 8);	// [15...8]
	data[cnt++] = (uint8_t) (alt_fu >> 16);	// [23...16]
	data[cnt++] = (uint8_t) (alt_fu >> 24);	// [31...24]

	data[cnt++] = (uint8_t) alt_add;		// [7...0]
	data[cnt++] = (uint8_t) (alt_add >> 8);	// [15...8]
	data[cnt++] = (uint8_t) (alt_add >> 16);	// [23...16]
	data[cnt++] = (uint8_t) (alt_add >> 24);	// [31...24]
	
	data[cnt++] = 1;

	for(i=0; i < (data[3]+4); i++)
	{
	 sc += data[i]; 									// CRCУ��1
	 ac += sc; 												// CRCУ��2
	}
	
	data[cnt++]=sc;
	data[cnt++]=ac;

	PrintDataApi(data, cnt);
}

/*
 * ��4·PWM���η��͵���λ��
 * ͨ��Э�飺����ͨ��Э��
 */
void SendFourPwmDuty(uint16_t pwm[4])
{
	uint8_t cnt=0;
	uint8_t i;
	uint8_t sc=0;
	uint8_t ac=0;
	
	uint8_t data[32] = {0};
	
	data[cnt++] = 0xAA;							// ֡ͷ 0xAA
	data[cnt++] = 0xFF;							// Ŀ���ַ 0xFF
	data[cnt++] = 0x20;							// ������
	data[cnt++] = 8;								// ���ݳ���
	

	data[cnt++] = (uint8_t) pwm[0];		// [7...0]
	data[cnt++] = (uint8_t) (pwm[0] >> 8);	// [15...8]

	data[cnt++] = (uint8_t) pwm[1];		// [7...0]
	data[cnt++] = (uint8_t) (pwm[1] >> 8);	// [15...8]

	data[cnt++] = (uint8_t) pwm[2];		// [7...0]
	data[cnt++] = (uint8_t) (pwm[2] >> 8);	// [15...8]

	data[cnt++] = (uint8_t) pwm[3];		// [7...0]
	data[cnt++] = (uint8_t) (pwm[3] >> 8);	// [15...8]

	for(i=0; i < (data[3]+4); i++)
	{
	 sc += data[i]; 									// CRCУ��1
	 ac += sc; 												// CRCУ��2
	}
	
	data[cnt++]=sc;
	data[cnt++]=ac;

	PrintDataApi(data, cnt);
}



/*************************************************************************/
/***************************** ���ղ��� ***********************************/
/*
 * ͨ��֡У��
 */
static uint8_t CheckFrame(const uint8_t *buf)
{
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t i;
	
	if (buf[0] != COMM_HEAD || buf[1] != COMM_D_ADDR)		// �ײ���Ŀ���ַ����
		return false;
	
	for (i=0; i<buf[3]+4; i++)
	{
		sumcheck += buf[i];
		addcheck += sumcheck;
	}
	
	if (sumcheck == buf[buf[3]+4] && addcheck == buf[buf[3]+5])
		return true;
	else
		return false;
}


/*
 * ң��������֡���
 */
void RcCmd_Buf2Frame(const uint8_t *buf, struct RC_CMD_t *frame)
{
	if (CheckFrame(buf) == true)																					// У��
		memcpy(frame, buf, sizeof(struct RC_CMD_t));												// ����
	frame -> data[RC_CMD_LEN-1] = 0;																			// ״̬λ���㣬˵���Ѿ������
}


