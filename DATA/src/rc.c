#include "rc.h"
#include "atti_esti.h"
#include "nrf.h"
#include "FreeRTOS.h"
#include "task.h"



static const int16_t rc_min[4] 	= {0, 			0, 		1, 		0};										// 遥控器通道最小值
static const int16_t rc_mid[4] 	= {2046, 	2061, 2076, 2066};									// 遥控器通道中间值
static const int16_t rc_max[4] 	= {4093, 	4092, 4092, 4092};									// 遥控器通道最大值
static const int16_t ch_dead 		= 10;																					// 遥控器死区


enum FLYMODE_e 		flymode_ 		= FLY_STABILIZE;			// 飞行模式
enum LOCKSTATE_e	lockstate_ 	= FLY_LOCKED;					// 解锁状态
struct RC_CMD_t 	rc_cmd_ 		= {0};								// 遥控器通信协议帧
struct RC_t 			rc_ 				= {0};								// 遥控器数据帧
struct RCM_t 			rcm_ 				= {0};								// 遥控器映射值



/*
 * NRF任务
 */
void RcTask(void *arg)
{
	uint8_t rx_buf[33];														// 接收缓存数组

	NRF24L01_Init();    		    									// 初始化NRF24L01 
	NRF24L01_RX_Mode();														// 接收模式
	
	while (1)
	{
		if(NRF24L01_RxPacket(rx_buf)==0)						// 接收到信息
		{
			switch (rx_buf[2])
			{
				case RC_CMD_ID:													// 遥控器命令帧
					RcCmd_Buf2Frame(rx_buf, &rc_cmd_);		// 转化为格式帧
					RcCmd2Rc(&rc_cmd_, &rc_);							// 转化为常用遥控器数据帧
					SetLockState(rc_, &lockstate_);				// 设置锁定状态
					SetFlyMode(rc_, &flymode_);						// 设置飞行模式
					RcMapping(rc_, &rcm_);								// 映射为期望值
				break;
				
				default:
				break;
			}
		}
		
		vTaskDelay(3);
	}
}




/*
 * 将遥控器原始数据解析成各通道数据
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
 * 遥控器数据映射
 * 通道1 -- 滚转角, vy
 * 通道2 -- 俯仰角, vx
 * 通道3 -- 油门, vz
 * 通道4 -- 偏航角速度
 * 通道5 -- 挡位
 * 通道6 -- 挡位
 * 通道7 -- 挡位
 * 通道8 -- 挡位
 */
static void RcMapping(struct RC_t rc, struct RCM_t *rcm)
{
	/* CH1 线性映射，保证遥控器CH2右推无人机要向右 */
	if (rc.joy.roll < rc_mid[0] - ch_dead)  			// 摇杆往右打，数值逐渐增大，期望滚转角逐渐增大
	{		
		rc.joy.roll += ch_dead;
		rcm->roll = LinearMapping(rc_mid[0], rc_min[0], 0, -RC_ROLL_MAX, rc.joy.roll);		// 映射关系：[摇杆中间, 摇杆最小] --> [0, 最小滚转角]
		rcm->vy = LinearMapping(rc_mid[0], rc_min[0], 0, -RC_VELY_MAX, rc.joy.roll);
	}
	else if (rc.joy.roll > rc_mid[0] + ch_dead)// 摇杆往左打，数值逐渐减小，期望滚转角逐渐减小（如：从0到-30°）
	{
		rc.joy.roll -= ch_dead;
		rcm->roll = LinearMapping(rc_mid[0], rc_max[0], 0, RC_ROLL_MAX, rc.joy.roll);		// 映射关系：[摇杆中间, 摇杆最大] --> [0, 最大滚转角]		
		rcm->vy = LinearMapping(rc_mid[0], rc_max[0], 0, RC_VELY_MAX, rc.joy.roll);
	}		
	else
	{
		rcm->roll = 0.0f;
		rcm->vy = 0.0f;
	}

	
	/* CH2 线性映射，保证遥控器CH2上推无人机要向前 */
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
	
	/* CH3 线性映射，保证遥控器CH3上推油门增大*/
	rcm->thr = LinearMapping(rc_min[2], rc_max[2], RC_THR_MIN, RC_THR_MAX, rc.joy.thr);
	
	if (rc.joy.thr < rc_mid[2] - ch_dead)
	{
		rc.joy.thr += ch_dead;
		rcm->vz = LinearMapping(rc_mid[2], rc_min[2], 0, RC_VELZ_MAX, rc.joy.thr); // 前右下坐标系，向下为 z正
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

	/* CH4 线性映射，保证遥控器CH4右推无人机顺时针旋转 */
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
 * 设置上锁与解锁
 */
static void SetLockState(const struct RC_t rc, enum LOCKSTATE_e *lockstate)
{	
	if ( fabs(rc.joy.roll - rc_min[0]) < 100 			// 内八解锁
			&& fabs(rc.joy.pitch - rc_min[1]) < 100
			&& fabs(rc.joy.thr - rc_min[2]) < 100
			&& fabs(rc.joy.yaw - rc_max[3]) < 100)
		*lockstate = FLY_UNLOCK;
	
	else if ( fabs(rc.joy.roll - rc_max[0]) < 100 			// 内八解锁
			&& fabs(rc.joy.pitch - rc_min[1]) < 100
			&& fabs(rc.joy.thr - rc_min[2]) < 100
			&& fabs(rc.joy.yaw - rc_min[3]) < 100)
		*lockstate = FLY_LOCKED;				
}

/*
 * 设置飞行模式
 */
static void SetFlyMode(const struct RC_t rc, enum FLYMODE_e *flymode)
{
	if (rc.key.key4 == 1)
		*flymode = FLY_ALTHOLD;		// 定高
	else
		*flymode = FLY_STABILIZE;
}





/*
 * 获取遥控器数据接口
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
 * 获取遥控器映射值
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
 * 获取飞行模式
 */
enum FLYMODE_e GetFlyModeApi(void)
{
	return flymode_;
}

/*
 * 获取上锁解锁状态
 */
enum LOCKSTATE_e GetLockStateApi(void)
{
	return lockstate_;
}

struct RC_t *PointRcData(void)
{
	return &rc_;
}



