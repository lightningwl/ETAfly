#include "pos_esti.h"
#include "atti_esti.h"
#include "FreeRTOS.h"
#include "task.h"


#define ALTI_MIN_VALUE												1						// ��ͺ���
#define ALTI_MAX_VALUE												5000				// ��ߺ���



struct XYZ_t acce_ 			= {0,0,0};												// ����ϵ���ٶ�

struct XYZ_t pos_raw_ 	= {0,0,0};												// ԭʼ������λ��
struct XYZ_t pos_fuse_ 	= {0,0,0};												// �ںϺ�λ��
struct XYZ_t vel_fuse_ 	= {0,0,0};												// �ںϺ��ٶ�



/*
 * λ�ù�������
 */
void PosEstiTask(void *arg)
{
	struct XYZ_t accb = {0,0,0};																				// ����ϵ���ٶ�
	struct XYZ_t accb_bias = {0,0,0};
	struct XYZ_t acce = {0,0,0};																				// ����ϵ���ٶ�
	struct XYZ_t pos0 = {0,0,0};																				// ��ʼλ��
		
	uint8_t count = 0;
	uint8_t times = 0;
	float alti = 0;
	portTickType t_last;
	
	vTaskDelay(1000);
	t_last = xTaskGetTickCount();																			// ��ȡ��ʼʱ��	
	
	
	while (1)
	{
		vTaskDelayUntil(&t_last, 1);
		
		accb = GetDof6LpfDataApi().acc;
		accb.x -= accb_bias.x;
		accb.y -= accb_bias.y;
		accb.z -= accb_bias.z;
		
		acce = AccBody2Earth(&accb);																			// ����ϵ���ٶ�
		AcceMean10(&acce, &acce_);																				// ���ٶ�ȡ��ֵ
		
		if (count % 10 == 0)
		{
			alti = GetDof10RawDataApi().alti;
			if (times < 100 && alti> ALTI_MIN_VALUE && alti < ALTI_MAX_VALUE)							// ��֤������Ч
			{
				pos0.z += alti;
				times ++;
			}
			else if (times == 100)
			{
				pos0.z /= 100.0f;																														// ��ʼ���θ߶�
				times ++;
			}
			else
			{
				pos_raw_.z = pos0.z - alti;																									// ��ѹ�Ʋ���z��λ��
				PoszEsti(0.01f, acce_.z, pos_raw_.z, 1, 1e-5f, &pos_fuse_.z, &vel_fuse_.z);	// z��λ���ٶȹ���
				accb_bias = GetAccBias(0.01f, pos_raw_.z, pos_fuse_.z);
			}
		}
		count ++;
		count %= 100;		
	}
}

/*
 * ����ϵ���ٶ�ƫ��У׼
 * �����ٶȽ�С��С��ACC_CORRECT_THRESHOLDʱ��ȡһ��ʱ����ƽ��ֵ��Ϊƫ��
 */
static struct XYZ_t GetAccBias(float dt, float p, float p_km)
{
	float e_pz = p - p_km;

	struct XYZ_t bias_b;
	struct XYZ_t bias_e;

	static float e_sum = 0;
	
	e_sum += e_pz * dt;
	
	bias_e.x = 0.0f;
	bias_e.y = 0.0f;
	bias_e.z = 0.05f * e_pz + 0.005f * e_sum;
	
	bias_b = VectorEarth2Body(&bias_e);
	
	return bias_b;
}



/*
 * z����λ���ٶȹ���
 *
 * ���룺���ٶ� acc
 * 			 λ�� pos
 * ���������λ�� pos_esti
 *			 �����ٶ� vel_esti
 */
static void PoszEsti(const float dt, const float acc, const float pos, 
	const float R, const float Q, 
	float *pos_esti, float *vel_esti)
{
	static float32_t Rz;
	static float32_t Qz[4];
	
	static float32_t pvz[2] 	= {0.0f, 0.0f};
	static float32_t Pz[4] 		= {1.0f, 0.0f, 0.0f, 0.1f};	
	
	static arm_matrix_instance_f32 _Pz;
	static arm_matrix_instance_f32 _Qz;
	static arm_matrix_instance_f32 _pvz;	

	static uint8_t times = 0;
	
	if (times == 0)
	{
		Rz = R;
		Qz[0] = Q;
		Qz[1] = 0;
		Qz[2] = 0;
		Qz[3] = Q;
		
		arm_mat_init_f32(&_Qz, 2, 2, Qz);
		arm_mat_init_f32(&_Pz, 2, 2, Pz);
		arm_mat_init_f32(&_pvz, 2, 1, pvz);			

		times = 1;
	}
	
	KmPosEsti(dt, acc, pos, Rz, _Qz, _Pz, _pvz);
	
	*pos_esti = _pvz.pData[0];
	*vel_esti = _pvz.pData[1];
}


/*
 * ��ֵ�˲�������3�μ��ٶ�ƽ��ֵ
 */
static void AcceMean10(const struct XYZ_t *acc, struct XYZ_t *acc_mean)
{
	#define LEN 	20
	static float accx[LEN] = {0,0,0,0,0, 0,0,0,0,0};
	static float accy[LEN] = {0,0,0,0,0, 0,0,0,0,0};
	static float accz[LEN] = {0,0,0,0,0, 0,0,0,0,0};
	
	static uint8_t count = 0;
		
	count ++;
	count %= LEN;
	
	accx[count] = acc->x;
	accy[count] = acc->y;
	accz[count] = acc->z;
	
	acc_mean->x = Mean(accx, LEN);
	acc_mean->y = Mean(accy, LEN);
	acc_mean->z = Mean(accz, LEN);
		
	#undef LEN
}


/*
 * ����ϵ���ٶ�ת��������ϵ
 */
static struct XYZ_t AccBody2Earth(const struct XYZ_t *b)
{
	struct XYZ_t e	= VectorBody2Earth(b);
	e.x = -e.x;
	e.y = -e.y;
	e.z = GRAVITY - e.z;
	
	return e;
}


static void KmPosEsti(float dt, 	// ʱ����
								float a,		// ����ϵ�¼��ٶ� (m/s^2)
								float p,		// ��ѹ�ƻ�GPS����λ�� (m)
								const float R,
								const arm_matrix_instance_f32 _Q,
								arm_matrix_instance_f32 _P,
								arm_matrix_instance_f32 _x
								)
{
	/**********************************************************/
	/* �������˲�������  */
	// km1
	float32_t H[2] = {1, 0};
	float32_t A[4];
	float32_t B[2];
	float32_t v2x1[2];
	float32_t x_[2];
	// km2
	float32_t T[4]; // ��ʱ�����м����
	float32_t T2[4];
	float32_t A_T[4];
	float32_t P_[4];
	// km3
	float32_t T3[2];
	float32_t H_T[2];
	float32_t T4[1];
	float32_t K[2];
	// km4

	// km5
	float32_t I[4] = {1, 0, 0, 1};
	
	/**********************************************************/
	/* ָ������ָ��	*/
	// km1
	arm_matrix_instance_f32 _H;
	arm_matrix_instance_f32 _A;
	arm_matrix_instance_f32 _B;
	arm_matrix_instance_f32 _v2x1;
	arm_matrix_instance_f32 _x_;
	
	// km2
	arm_matrix_instance_f32 _A_T;	
	arm_matrix_instance_f32 _T;
	arm_matrix_instance_f32 _T2;
	arm_matrix_instance_f32 _P_;
	// km3
	arm_matrix_instance_f32 _T3;
	arm_matrix_instance_f32 _H_T;
	arm_matrix_instance_f32 _T4;
	arm_matrix_instance_f32 _K;
	// km4
	// km5
	arm_matrix_instance_f32 _I;
	
	/**********************************************************/
	/* ��ʼ����ָ�������ƥ�� */
	// km1
	arm_mat_init_f32(&_H, 1, 2, H);
	arm_mat_init_f32(&_A, 2, 2, A);
	arm_mat_init_f32(&_B, 2, 1, B);
	arm_mat_init_f32(&_v2x1, 2, 1, v2x1);
	arm_mat_init_f32(&_x_, 2, 1, x_);
	// km2
	arm_mat_init_f32(&_A_T, 2, 2, A_T);
	arm_mat_init_f32(&_T, 2, 2, T);
	arm_mat_init_f32(&_T2, 2, 2, T2);
	arm_mat_init_f32(&_P_, 2, 2, P_);
	// km3
	arm_mat_init_f32(&_T3, 1, 2, T3);
	arm_mat_init_f32(&_H_T, 2, 1, H_T);
	arm_mat_init_f32(&_T4, 1, 1, T4);
	arm_mat_init_f32(&_K, 2, 1, K);
	// km4
	// km5
	arm_mat_init_f32(&_I, 2, 2, I);
	
	/**********************************************************/
	/* ��������ʽ1	*/
	A[0] = 1;  A[1] = dt;
	A[2] = 0;  A[3] = 1;
	
	B[0] = 0.5f * dt * dt;
	B[1] = dt;
	
	arm_mat_mult_f32(&_A, &_x, &_v2x1);
	B[0] *= a;
	B[1] *= a;
	arm_mat_add_f32(&_v2x1, &_B, &_x_);
		
	// ��������ʽ2
	arm_mat_trans_f32(&_A, &_A_T);					// A' (2x2)
	arm_mat_mult_f32(&_A, &_P, &_T);				// T = A*P  (2x2)
	arm_mat_mult_f32(&_T, &_A_T, &_T2);			// T2 = A*P*A'  (2x2)
	arm_mat_add_f32(&_T2, &_Q, &_P_);				// P_ = A*P*A' + Q (2x2)
	
	// ��������ʽ3
	arm_mat_mult_f32(&_H, &_P_, &_T3);			// T = H*P_		(1x2)
	arm_mat_trans_f32(&_H, &_H_T);					// H' (2x1)
	arm_mat_mult_f32(&_T3, &_H_T, &_T4);		// T4 = H*P_*H' (1x1)
	T4[0] += R;															//arm_mat_add_f32(&_T4, &_R, &_T4);			//  H*P_*H' + R				 (1x1)
	T4[0] = 1 / T4[0];											//arm_mat_inverse_f32(&_M, &_M_);				// inv(H*P_*H' + R)		 (3x3)
	arm_mat_init_f32(&_T3, 2, 1, T3);				// ��T3��ʼ��Ϊ 3x1 ����
	arm_mat_mult_f32(&_P_, &_H_T, &_T3);		// T = P_ * H' (2x1)
	arm_mat_mult_f32(&_T3, &_T4, &_K);			// K = P_ * H' * inv(H1 * P_ * H' + R);  (2x1)
		
	// ��������ʽ4
	arm_mat_mult_f32(&_H, &_x_, &_T4);			// T4 = H*x (1x1)
	T4[0] = p - T4[0];											// T4 = p - H*x (1x1)
	arm_mat_mult_f32(&_K, &_T4, &_v2x1);		// v2x1 = K * (p - H*x)	  (2x1)
	arm_mat_add_f32(&_x_, &_v2x1, &_x);			// x = x_ + K * (p - H*x)	 (2x1)

	// ��������ʽ5
	arm_mat_mult_f32(&_K, &_H, &_T);				// T = K*H  (2x2)
	arm_mat_sub_f32(&_I, &_T, &_T2);				// T2 = I - K*H (2x2)
	arm_mat_mult_f32(&_T2, &_P_, &_P);			// P = (I - K*H) * P_
}


/*
 * λ���ٶȹ���
 */
static void KmPosVelEsti(float dt, 	// ʱ����
								float a,		// ����ϵ�¼��ٶ� (m/s^2)
							  float v,    // ����ϵ���ٶ�(m/s)
								float p,		// ��ѹ�ƻ�GPS����λ�� (m)
								const arm_matrix_instance_f32 _R,
								const arm_matrix_instance_f32 _Q,
								arm_matrix_instance_f32 _P,
								arm_matrix_instance_f32 _x
								)
{
	/**********************************************************/
	// km1
	float32_t H[4] = {1, 0, 0, 1};
	float32_t A[4];
	float32_t B[2];
	float32_t v2x1[2];
	float32_t x_[2];
	// km2
	float32_t T[4]; // ��ʱ�����м����
	float32_t T2[4];
	float32_t A_T[4];
	float32_t P_[4];
	// km3
	float32_t H_T[4];
	float32_t T4[4];
	float32_t K[4];
	// km4
	float32_t T3[2];

	// km5
	float32_t I[4] = {1, 0, 0, 1};
	/**********************************************************/
	// km1
	arm_matrix_instance_f32 _H;
	arm_matrix_instance_f32 _A;
	arm_matrix_instance_f32 _B;
	arm_matrix_instance_f32 _v2x1;
	arm_matrix_instance_f32 _x_;
	
	// km2
	arm_matrix_instance_f32 _A_T;	
	arm_matrix_instance_f32 _T;
	arm_matrix_instance_f32 _T2;
	arm_matrix_instance_f32 _P_;
	// km3
	arm_matrix_instance_f32 _H_T;
	arm_matrix_instance_f32 _T4;
	arm_matrix_instance_f32 _K;
	// km4
	arm_matrix_instance_f32 _T3;
	// km5
	arm_matrix_instance_f32 _I;
	/**********************************************************/
	// km1
	arm_mat_init_f32(&_H, 2, 2, H);
	arm_mat_init_f32(&_A, 2, 2, A);
	arm_mat_init_f32(&_B, 2, 1, B);
	arm_mat_init_f32(&_v2x1, 2, 1, v2x1);
	arm_mat_init_f32(&_x_, 2, 1, x_);
	// km2
	arm_mat_init_f32(&_A_T, 2, 2, A_T);
	arm_mat_init_f32(&_T, 2, 2, T);
	arm_mat_init_f32(&_T2, 2, 2, T2);
	arm_mat_init_f32(&_P_, 2, 2, P_);
	// km3
	arm_mat_init_f32(&_T3, 2, 1, T3);
	arm_mat_init_f32(&_H_T, 2, 2, H_T);
	arm_mat_init_f32(&_T4, 2, 2, T4);
	arm_mat_init_f32(&_K, 2, 2, K);
	// km4
	// km5
	arm_mat_init_f32(&_I, 2, 2, I);
	/**********************************************************/
	// ��������ʽ1
	A[0] = 1;  A[1] = dt;
	A[2] = 0;  A[3] = 1;
	
	B[0] = 0.5f * dt * dt;
	B[1] = dt;
	
	arm_mat_mult_f32(&_A, &_x, &_v2x1);
	B[0] *= a;
	B[1] *= a;
	arm_mat_add_f32(&_v2x1, &_B, &_x_);
		
	// ��������ʽ2
	arm_mat_trans_f32(&_A, &_A_T);					// A' (2x2)
	arm_mat_mult_f32(&_A, &_P, &_T);				// T = A*P  (2x2)
	arm_mat_mult_f32(&_T, &_A_T, &_T2);			// T2 = A*P*A'  (2x2)
	arm_mat_add_f32(&_T2, &_Q, &_P_);				// P_ = A*P*A' + Q (2x2)
	
	//printM("P_ = ", _P_);
	// ��������ʽ3
	arm_mat_mult_f32(&_H, &_P_, &_T2);			// T = H*P_		(2x2)
	arm_mat_trans_f32(&_H, &_H_T);					// H' (2x2)
	arm_mat_mult_f32(&_T2, &_H_T, &_T4);			// T4 = H*P_*H' (2x2)
	arm_mat_add_f32(&_T4, &_R, &_T4);			//  H*P_*H' + R				 (2x2)
	arm_mat_inverse_f32(&_T4, &_T);				// inv(H*P_*H' + R)		 (2x2)
	
	arm_mat_mult_f32(&_P_, &_H_T, &_T2);		// T2 = P_ * H' (2x2)
	arm_mat_mult_f32(&_T2, &_T, &_K);				// K = P_ * H' * inv(H1 * P_ * H' + R);  (2x2)

	// ��������ʽ4
	arm_mat_mult_f32(&_H, &_x_, &_v2x1);			// v2x1 = H*x (2x1)
	v2x1[0] = p - v2x1[0];										// v2x1 = [p;v] - H*x (2x1)
	v2x1[1] = v - v2x1[1];
	arm_mat_mult_f32(&_K, &_v2x1, &_T3);		// T3 = K * (p - H*x)	  (2x1)
	arm_mat_add_f32(&_x_, &_T3, &_x);			// x = x_ + K * (p - H*x)	 (2x1)

	// ��������ʽ5
	arm_mat_mult_f32(&_K, &_H, &_T);			// T = K*H  (2x2)
	arm_mat_sub_f32(&_I, &_T, &_T2);			// T2 = I - K*H (2x2)
	arm_mat_mult_f32(&_T2, &_P_, &_P);		// P = (I - K*H) * P_
}




/*
 * ��ȡ����ϵ�¼��ٶ�
 */
struct XYZ_t *PointEarthAccApi(void)
{
	return &acce_;
}


/*
 * ��ȡԭʼ��λ����
 */
struct XYZ_t GetRawPosApi(void)
{
	return pos_raw_;
}

struct XYZ_t *PointRawPosApi(void)
{
	return &pos_raw_;
}

/*
 * ��ȡ�ںϵ�λ��
 */

struct XYZ_t GetFusePosApi(void)
{
	return pos_fuse_;
}

struct XYZ_t *PointFusePosApi(void)
{
	return &pos_fuse_;
}

/*
 * ��ȡ�ںϺ���ٶ�
 */
struct XYZ_t GetFuseVelApi(void)
{
	return vel_fuse_;
}

struct XYZ_t *PointFuseVelApi(void)
{
	return &vel_fuse_;
}
