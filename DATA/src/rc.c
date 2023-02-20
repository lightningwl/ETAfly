#include "rc.h"
#include "atti_esti.h"
#include "nrf.h"
#include "FreeRTOS.h"
#include "task.h"



static const int16_t rc_min[4] 	= {0, 			0, 		1, 		0};										// ң����ͨ����Сֵ
static const int16_t rc_mid[4] 	= {2046, 	2061, 2076, 2066};									// ң����ͨ���м�ֵ
static const int16_t rc_max[4] 	= {4093, 	4092, 4092, 4092};									// ң����ͨ�����ֵ
static const int16_t ch_dead 		= 10;																					// ң��������


enum FLYMODE_e 		flymode_ 		= FLY_STABILIZE;			// ����ģʽ
enum LOCKSTATE_e	lockstate_ 	= FLY_LOCKED;					// ����״̬
struct RC_CMD_t 	rc_cmd_ 		= {0};								// ң����ͨ��Э��֡
struct RC_t 			rc_ 				= {0};								// ң��������֡
struct RCM_t 			rcm_ 				= {0};								// ң����ӳ��ֵ



/*
 * NRF����
 */
void RcTask(void *arg)
{
	uint8_t rx_buf[33];														// ���ջ�������

	NRF24L01_Init();    		    									// ��ʼ��NRF24L01 
	NRF24L01_RX_Mode();														// ����ģʽ
	
	while (1)
	{
		if(NRF24L01_RxPacket(rx_buf)==0)						// ���յ���Ϣ
		{
			switch (rx_buf[2])
			{
				case RC_CMD_ID:													// ң��������֡
					RcCmd_Buf2Frame(rx_buf, &rc_cmd_);		// ת��Ϊ��ʽ֡
					RcCmd2Rc(&rc_cmd_, &rc_);							// ת��Ϊ����ң��������֡
					SetLockState(rc_, &lockstate_);				// ��������״̬
					SetFlyMode(rc_, &flymode_);						// ���÷���ģʽ
					RcMapping(rc_, &rcm_);								// ӳ��Ϊ����ֵ
				break;
				
				default:
				break;
			}
		}
		
		vTaskDelay(3);
	}
}




/*
 * ��ң����ԭʼ���ݽ����ɸ�ͨ������
 */
void RcCmd2Rc(const struct RC_CMD_t *cmd, struct RC_t *rc)
{
	
	rc->joy.roll    = cmd->data[0] << 8 | cmd->data[1];
	rc->joy.pitch   = cmd->data[2] << 8 | cmd->data[3];
	rc->joy.thr     = cmd->data[4] << 8 | cmd->data[5];
	rc->joy.yaw 		= cmd->data[6] << 8 | cmd->data[7];
	
	rc->key.key1		= (cmd->data[8] >> 0) & 0x01;
	rc->key.key2		= (cmd->data[8] >> 1) & 0x01;
	rc->key.key3		= (cmd->data[8] >> 2) & 0x01;
	rc->key.key4		= (cmd->data[8] >> 3) & 0x01;
}


/*
 * ң��������ӳ��
 * ͨ��1 -- ��ת��, vy
 * ͨ��2 -- ������, vx
 * ͨ��3 -- ����, vz
 * ͨ��4 -- ƫ�����ٶ�
 * ͨ��5 -- ��λ
 * ͨ��6 -- ��λ
 * ͨ��7 -- ��λ
 * ͨ��8 -- ��λ
 */
static void RcMapping(struct RC_t rc, struct RCM_t *rcm)
{
	/* CH1 ����ӳ�䣬��֤ң����CH2�������˻�Ҫ���� */
	if (rc.joy.roll < rc_mid[0] - ch_dead)  			// ҡ�����Ҵ���ֵ������������ת��������
	{		
		rc.joy.roll += ch_dead;
		rcm->roll = LinearMapping(rc_mid[0], rc_min[0], 0, -RC_ROLL_MAX, rc.joy.roll);		// ӳ���ϵ��[ҡ���м�, ҡ����С] --> [0, ��С��ת��]
		rcm->vy = LinearMapping(rc_mid[0], rc_min[0], 0, -RC_VELY_MAX, rc.joy.roll);
	}
	else if (rc.joy.roll > rc_mid[0] + ch_dead)// ҡ���������ֵ�𽥼�С��������ת���𽥼�С���磺��0��-30�㣩
	{
		rc.joy.roll -= ch_dead;
		rcm->roll = LinearMapping(rc_mid[0], rc_max[0], 0, RC_ROLL_MAX, rc.joy.roll);		// ӳ���ϵ��[ҡ���м�, ҡ�����] --> [0, ����ת��]		
		rcm->vy = LinearMapping(rc_mid[0], rc_max[0], 0, RC_VELY_MAX, rc.joy.roll);
	}		
	else
	{
		rcm->roll = 0.0f;
		rcm->vy = 0.0f;
	}

	
	/* CH2 ����ӳ�䣬��֤ң����CH2�������˻�Ҫ��ǰ */
	if (rc.joy.pitch < rc_mid[1] - ch_dead)
	{
		rc.joy.pitch += ch_dead;
		rcm->pitch = -LinearMapping(rc_mid[1], rc_min[1], 0, -RC_PITCH_MAX, rc.joy.pitch);
		rcm->vx = -LinearMapping(rc_mid[1], rc_min[1], 0, RC_VELX_MAX, rc.joy.pitch);
	}
	else if (rc.joy.pitch > rc_mid[1] + ch_dead)
	{
		rc.joy.pitch -= ch_dead;
		rcm->pitch = -LinearMapping(rc_mid[1], rc_max[1], 0, RC_PITCH_MAX, rc.joy.pitch);
		rcm->vx = -LinearMapping(rc_mid[1], rc_max[1], 0, -RC_VELX_MAX, rc.joy.pitch);
	}
	else
	{
		rcm->pitch = 0.0f;
		rcm->vx = 0.0f;
	}
	
	/* CH3 ����ӳ�䣬��֤ң����CH3������������*/
	rcm->thr = LinearMapping(rc_min[2], rc_max[2], RC_THR_MIN, RC_THR_MAX, rc.joy.thr);
	
	if (rc.joy.thr < rc_mid[2] - ch_dead)
	{
		rc.joy.thr += ch_dead;
		rcm->vz = LinearMapping(rc_mid[2], rc_min[2], 0, RC_VELZ_MAX, rc.joy.thr); // ǰ��������ϵ������Ϊ z��
	}
	else if (rc.joy.thr > rc_mid[2] + ch_dead)
	{
		rc.joy.thr -= ch_dead;
		rcm->vz = LinearMapping(rc_mid[2], rc_max[2], 0, -RC_VELZ_MAX, rc.joy.thr);  
	}
	else
	{
		rcm->vz = 0.0f;
	}

	/* CH4 ����ӳ�䣬��֤ң����CH4�������˻�˳ʱ����ת */
	if (rc.joy.yaw < rc_mid[3] - ch_dead)
	{
		rc.joy.yaw += ch_dead;
		rcm->yaw_rate = LinearMapping(rc_mid[3], rc_min[3], 0, -RC_YAW_RATE_MAX, rc.joy.yaw);
	}
	else if (rc.joy.yaw > rc_mid[3] + ch_dead)
	{
		rc.joy.yaw -= ch_dead;
		rcm->yaw_rate = LinearMapping(rc_mid[3], rc_max[3], 0, RC_YAW_RATE_MAX, rc.joy.yaw);	
	}
	else
	{
		rcm->yaw_rate = 0.0f;
	}	
	
	/* CH5-CH8 */
	rcm->ch5 = rc.key.key1;
	rcm->ch6 = rc.key.key2;
	rcm->ch7 = rc.key.key3;
	rcm->ch8 = rc.key.key4;
}



/*
 * �������������
 */
static void SetLockState(const struct RC_t rc, enum LOCKSTATE_e *lockstate)
{	
	if ( fabs(rc.joy.roll - rc_min[0]) < 100 			// �ڰ˽���
			&& fabs(rc.joy.pitch - rc_min[1]) < 100
			&& fabs(rc.joy.thr - rc_min[2]) < 100
			&& fabs(rc.joy.yaw - rc_max[3]) < 100)
		*lockstate = FLY_UNLOCK;
	
	else if ( fabs(rc.joy.roll - rc_max[0]) < 100 			// �ڰ˽���
			&& fabs(rc.joy.pitch - rc_min[1]) < 100
			&& fabs(rc.joy.thr - rc_min[2]) < 100
			&& fabs(rc.joy.yaw - rc_min[3]) < 100)
		*lockstate = FLY_LOCKED;				
}

/*
 * ���÷���ģʽ
 */
static void SetFlyMode(const struct RC_t rc, enum FLYMODE_e *flymode)
{
	if (rc.key.key4 == 1)
		*flymode = FLY_ALTHOLD;		// ����
	else
		*flymode = FLY_STABILIZE;
}





/*
 * ��ȡң�������ݽӿ�
 */
struct RC_t GetRcApi(void)
{
	return rc_;
}

struct RC_t *PointRcApi(void)
{
	return &rc_;
}

/*
 * ��ȡң����ӳ��ֵ
 */
struct RCM_t GetRcmApi(void)
{
	return rcm_;
}

struct RCM_t *PointRcmApi(void)
{
	return &rcm_;
}


/*
 * ��ȡ����ģʽ
 */
enum FLYMODE_e GetFlyModeApi(void)
{
	return flymode_;
}

/*
 * ��ȡ��������״̬
 */
enum LOCKSTATE_e GetLockStateApi(void)
{
	return lockstate_;
}

struct RC_t *PointRcData(void)
{
	return &rc_;
}



