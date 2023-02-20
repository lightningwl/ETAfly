/*
 * �������˲���̬����
 */

#include "FreeRTOS.h"
#include "task.h"
#include "atti_esti.h"
#include "uavmath.h"
#include "dof10.h"
#include "delay.h"


struct EULAR_t eular_  	= {0, 0.0f, 0.0f, 0.0f};								// ��ǰŷ����
struct QUAT_t quat_ 		= {0, 1.0f, 0.0f, 0.0f, 0.0f};					// ��ǰ��Ԫ��
struct ROTATE_t rot_		= {0};																	// ��ת����

struct EULAR_t eular0_  =	{0, 0.0f, 0.0f, 0.0f};								// ��ʼŷ����
struct QUAT_t quat0_ 		= {0, 0.0f, 0.0f, 0.0f, 0.0f};					// ��ʼ��Ԫ��
struct ROTATE_t rot0_		= {0};																	// ��ʼ��ת����


/*
 * ��̬��������
 */
void AttiEstiTask(void *arg)
{
	struct DOF9_t dof9;
	portTickType t_last;
	const float dt = 0.002f;
	uint8_t count = 0;
	
	t_last = xTaskGetTickCount();
	
	while (1)
	{
		vTaskDelayUntil(&t_last, 2);														// ��ʱ����, 500Hz
				
		dof9.acc = GetDof6LpfDataApi().acc;											// ��ȡ���ٶ�
		dof9.gyro = GetDof6LpfDataApi().gyro;										// ��ȡ���ٶ�
		Dof3Deg2Rad(&dof9.gyro);																// ���ٶ�ת��Ϊ����Ϊ��λ
		
		if (count % 5 == 0)
			dof9.mag = GetDof9CaliDataApi().mag;									// ��ȡ����������, 50Hz
		
		KmAttiEsti(dof9, dt);																		// �������˲���̬����, 500Hz 
		
		if (eular0_.valid != 1)																	// ��ȡ��ʼ����̬��
			GetInitAtti();
		
		count ++;
		count %= 100;
	}
}

/*
 * �������˲���̬����
 * ���룺���ᴫ�������� dof9�����ٶ� m/s^2, ���ٶ� rad/s, ������ IIC��ȡ�ų�ǿ��*0.01
 * �˺���������ȫ�ֱ��� ��̬��Ԫ�� quat_ �� ŷ���� eular_
 */
void KmAttiEsti(const struct DOF9_t dof9, float dt)
{
	/* �������˲���ؾ���		*/
	static float32_t Q[16] = {1e-4f,0,0,0,   0,1e-4f,0,0,  0,0,1e-4f,0,  0,0,0,1e-4f};	// ��������Э�������
	static float32_t R[9] = {1,0,0,  0,1,0,    0,0,1};																	// ���ٶȼƲ�������Э�������
	static float32_t R2[9] = {3,0,0,  0,3,0,    0,0,3};																	// �����Ʋ�������Э�������
	
	static float32_t q[4] = {1, 0, 0, 0};																								// ʹ�ü��ٶȼƺ���õ���Э�������
	static float32_t q2[4] = {1, 0, 0, 0};																							// ʹ�ô����ƺ���õ���Э�������
	static float32_t q_[4] = {1, 0, 0, 0};																							// �����q
	static float32_t P_[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};											// ���������Э�������
	static float32_t P1[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; 										// ͨ�����ٶȼƺ���õ���Э�������
	static float32_t P2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};											// ͨ���Ӵ����ƺ���õ���Э�������
	static float32_t K[12];																															// ����������
	static float32_t K2[12];																														// ����������
	
	
	/* ָ�򿨶����˲���ؾ����ָ��	*/
	static arm_matrix_instance_f32 _Q;
	static arm_matrix_instance_f32 _R;
	static arm_matrix_instance_f32 _R2;
	
	static arm_matrix_instance_f32 _q;
	static arm_matrix_instance_f32 _q2;
	static arm_matrix_instance_f32 _q_;
	static arm_matrix_instance_f32 _P_;
	static arm_matrix_instance_f32 _P1;
	static arm_matrix_instance_f32 _P2;
	static arm_matrix_instance_f32 _K;
	static arm_matrix_instance_f32 _K2;
		
	static uint16_t km_atti_times = 0;

	/* ��ʼ���������˲���ؾ���			*/
	
	if (km_atti_times == 0)
	{
		arm_mat_init_f32(&_Q, 4, 4, Q);
		arm_mat_init_f32(&_R, 3, 3, R);
		arm_mat_init_f32(&_R2, 3, 3, R2);
		
		arm_mat_init_f32(&_q, 4, 1, q);
		arm_mat_init_f32(&_q2, 4, 1, q2);
		arm_mat_init_f32(&_q_, 4, 1, q_);
		arm_mat_init_f32(&_P_, 4, 4, P_);
		arm_mat_init_f32(&_P1, 4, 4, P1);
		arm_mat_init_f32(&_P2, 4, 4, P2);
		arm_mat_init_f32(&_K, 4, 3, K);
		arm_mat_init_f32(&_K2, 4, 3, K2);	
		
		km_atti_times = 1;
	}
	else if (km_atti_times < 1000)										// �����μ��ٶȼƺʹ����ƣ�������
	{
		Q[0] = 1e-3f;
	  Q[5] = 1e-3f;
		Q[10] = 1e-3f;
		Q[15] = 1e-3f;
		km_atti_times ++;
	}
	else																							// �����������ǣ�׼ȷ
	{
		Q[0] = 1e-8f;
	  Q[5] = 1e-8f;
		Q[10] = 1e-8f;
		Q[15] = 1e-8f;		
	}
	
	/* �������˲�����	*/
	Km12(dof9.gyro, dt, _q2, _P2, _Q, _q_, _P_);			// ��������ʽ1,2 ����������Ԥ��
	Km3(0, _q2, _P_, _R, _K);													// ��������ʽ3�����㿨��������
	Km45(0, dof9.acc, _q_, _q2, _P_, _K, _q, _P1);		// ��������ʽ4,5 ���ݼ��ٶ�����
	Km3(1, _q2, _P_, _R2, _K2);												// ��������ʽ3�����㿨��������
	Km45(1, dof9.mag, _q, _q2, _P1, _K2, _q2, _P2);		// ��������ʽ4��5 ���ݴ���������
	
	NormalizeQuat(&_q2);															// ��Ԫ����һ��
	
	/* ��ֵ��ȫ�ֱ��� */
	quat_.q0 = _q2.pData[0];													// ��ֵ��ȫ�ֱ���
	quat_.q1 = _q2.pData[1];
	quat_.q2 = _q2.pData[2];
	quat_.q3 = _q2.pData[3];
	
	Quat2EularAngleDeg(_q2, &eular_);									// ת��Ϊŷ���ǣ���ֵ��ȫ�ֱ���
	Quat2Rbe(&quat_, rot_.R);														// ת��Ϊ��ת����
}

/* 
 * ��������ʽ1,2--���������q_��P_
 * ���룺������ٶ� w (rad/s), (3x1)
 * 			ʱ�䲽�� dt (s)
 *			��һ�κ������Ԫ�� q (4x1)
 *			��һ�κ����Э������� P (4x4)
 *			��������Э�������Q (4x4)
 * ���������״̬����Ԫ�� q_ (4x1)
 *			����Э������� P_  (4x4)
 */
static void Km12(const struct XYZ_t w, const float dt, const arm_matrix_instance_f32 _q, const arm_matrix_instance_f32 _P, const arm_matrix_instance_f32 _Q ,arm_matrix_instance_f32 _q_, arm_matrix_instance_f32 _P_)
{
	float32_t A[16];
	arm_matrix_instance_f32 _A;
	
	float32_t A_T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// ����A��ת��
	arm_matrix_instance_f32 _A_T;

	float32_t T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// ��ʱ�����м����
	arm_matrix_instance_f32 _T;
	
	float32_t T2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// ��ʱ�����м����
	arm_matrix_instance_f32 _T2;
	
	float temp;
	
	
	temp = dt / 2;

	A[0] = 1.0f; 			A[1] = -w.x*temp; 	A[2] = -w.y*temp; 	A[3] = -w.z*temp;
	A[4] = w.x*temp;	A[5] = 1.0f;				A[6] = w.z*temp;		A[7] = -w.y*temp;
	A[8] = w.y*temp;	A[9] = -w.z*temp;		A[10] = 1.0f;				A[11] = w.x*temp;
	A[12] = w.z*temp;	A[13]= w.y*temp;		A[14] = -w.x*temp;	A[15] = 1.0f;
	
	arm_mat_init_f32(&_A, 4, 4, A);				// ת�ƾ���A (4x4)
	arm_mat_init_f32(&_A_T, 4, 4, A_T);
	arm_mat_init_f32(&_T, 4, 4, T);
	arm_mat_init_f32(&_T2, 4, 4, T2);
	
	arm_mat_trans_f32(&_A, &_A_T);				// A' (4x4)
	arm_mat_mult_f32(&_A, &_P, &_T);			// T = A*P  (4x4)
	arm_mat_mult_f32(&_T, &_A_T, &_T2);		// P_ = A*P*A'  (4x4)
	arm_mat_add_f32(&_T2, &_Q, &_P_);
		
	arm_mat_mult_f32(&_A, &_q, &_q_);			// q_ = A*q (4x1)
}

/*
 * ��������ʽ3--���㿨��������
 * ���룺��һ�ι�����Ԫ�� q (4x1)
 *			���������Э������� P_ (4x4)
 *			��������Э������� R (3x3)
 * ��������������� K (4x3)
 */
static void Km3(const uint8_t flag, const arm_matrix_instance_f32 _q, const arm_matrix_instance_f32 _P_, const arm_matrix_instance_f32 _R, arm_matrix_instance_f32 _K)
{
	float32_t H[12];	// (3x4)
	float32_t H_T[12];// (4x3)
	float32_t T[12];// (3x4)
	float32_t T1[9];// (3x3)
	float32_t M[9];			// (3x3)
	float32_t M_[9];		// M����, (3x3)
	
	arm_matrix_instance_f32 _H;
	arm_matrix_instance_f32 _H_T;
	arm_matrix_instance_f32 _T;
	arm_matrix_instance_f32 _T1;
	arm_matrix_instance_f32 _M;
	arm_matrix_instance_f32 _M_;

	// �۲����
	if (flag == 0)
	{
		H[0] = -2*_q.pData[2];		H[1] =  2*_q.pData[3];			H[2] = -2*_q.pData[0];			H[3] =  2*_q.pData[1];
		H[4] =  2*_q.pData[1];		H[5] =  2*_q.pData[0];			H[6] =  2*_q.pData[3];			H[7] =  2*_q.pData[2];
		H[8] =  2*_q.pData[0];		H[9] = -2*_q.pData[1];			H[10] = -2*_q.pData[2];		H[11] =  2*_q.pData[3];
	}
	else if (flag == 1)
	{
		H[0] =  2*_q.pData[0];		H[1] =  2*_q.pData[1];			H[2] = -2*_q.pData[2];			H[3] = -2*_q.pData[3];
		H[4] = -2*_q.pData[3];		H[5] =  2*_q.pData[2];			H[6] =  2*_q.pData[1];			H[7] = -2*_q.pData[0];
		H[8] =  2*_q.pData[2];		H[9] =  2*_q.pData[3];			H[10] = 2*_q.pData[0];			H[11] = 2*_q.pData[1];
	}
	else
	{
		printf("Km3, neither acc or mag data.\r\n");
	}
	
	// �����ʼ��
	arm_mat_init_f32(&_H, 3, 4, H);				// �۲����H (3x4)
	arm_mat_init_f32(&_H_T, 4, 3, H_T);
	arm_mat_init_f32(&_T, 3, 4, T);				// T (3x4)
	arm_mat_init_f32(&_M, 3, 3, M);				// M (3x3)
	arm_mat_init_f32(&_M_, 3, 3, M_);			// M_(3x3)
	arm_mat_init_f32(&_T1, 3, 3, T1);
	
	// ����������
	arm_mat_mult_f32(&_H, &_P_, &_T);			// T = H*P_		(3x4)
	arm_mat_trans_f32(&_H, &_H_T);				// H' (4x3)
	arm_mat_mult_f32(&_T, &_H_T, &_T1);		// T1 = H*P_*H' (3x3)
	
	arm_mat_add_f32(&_T1, &_R, &_M);			// M = H*P_*H' + R				 (3x3)
	arm_mat_inverse_f32(&_M, &_M_);				// M_ = inv(H*P_*H' + R)		 (3x3)
	arm_mat_init_f32(&_T, 4, 3, T);				// ��T��ʼ��Ϊ 4x3 ����
	arm_mat_mult_f32(&_P_, &_H_T, &_T);		// T = P_ * H' (4x3)
	arm_mat_mult_f32(&_T, &_M_, &_K);			// K = P_ * H' * inv(H1 * P_ * H' + R);  (4x3)
}


/*
 * ��������ʽ4,5
 * ���룺�۲��״̬ a (���ٶȵ�λΪ N/s^2�� (3x1)
 *			�������Ԫ�� qi_ (4x1)
 *      ��һ�ι��Ƶ� q
 *			���������Э������� P_
 *			���������� K (4x3)
 * ������������Ԫ�� qo (4x1)
 *			���������Э������� P (4x4)
 */
static void Km45(const uint8_t flag, 
		const struct XYZ_t a, 
		const arm_matrix_instance_f32 _qi, 
		const arm_matrix_instance_f32 _q, 
		const arm_matrix_instance_f32 _P_, 
		const arm_matrix_instance_f32 _K, 
		arm_matrix_instance_f32 _qo, 
		arm_matrix_instance_f32 _P)
{
	float norm 	; 			// ģ��
	
	float32_t z[3];			// �۲������
	float32_t h[3];			// ����ϵ�����ڻ���ϵ�µķ��� (3x1)
	float32_t v3[3];		// ��ʱ���� (3x1)
	float32_t v4[4];		// ��ʱ������4x1��
	float32_t I[16]= {1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};		// ��λ��4x4��
	float32_t H[12];
	float32_t T[16];
	float32_t M[16];
	
	
	arm_matrix_instance_f32 _z;
	arm_matrix_instance_f32 _h;
	arm_matrix_instance_f32 _v3;
	arm_matrix_instance_f32 _v4;
	arm_matrix_instance_f32 _I ;
	arm_matrix_instance_f32 _H;
	arm_matrix_instance_f32 _T;
	arm_matrix_instance_f32 _M;
	
	// ��ȡ����ֵ
	z[0] = a.x;
	z[1] = a.y;
	z[2] = a.z;
	
	// �����ʼ��
	arm_mat_init_f32(&_z, 3, 1, z);
	arm_mat_init_f32(&_h, 3, 1, h);
	arm_mat_init_f32(&_v3, 3, 1, v3);
	arm_mat_init_f32(&_v4, 4, 1, v4);
	arm_mat_init_f32(&_I, 4, 4, I);
	arm_mat_init_f32(&_H, 3, 4, H);
	arm_mat_init_f32(&_T, 4, 4, T);
	arm_mat_init_f32(&_M, 4, 4, M);
	
	norm = sqrtf(a.x*a.x + a.y*a.y + a.z * a.z);
	if (fabs(norm > 1e-8f))
		norm = 1.0f / norm;
	else
		norm = GRAVITY;
	z[0] *= norm;
	z[1] *= norm;
	z[2] *= norm;
	
	// ����ֵ
	if (flag == 0)
	{
		h[0] = 2*_q.pData[1]*_q.pData[3] - 2*_q.pData[0]*_q.pData[2];
		h[1] = 2*_q.pData[0]*_q.pData[1] + 2*_q.pData[2]*_q.pData[3];
		h[2] = _q.pData[0]*_q.pData[0] - _q.pData[1]*_q.pData[1] - _q.pData[2]*_q.pData[2] + _q.pData[3]*_q.pData[3]; 
	
		// �۲����
		H[0] = -2*_q.pData[2];		H[1] =  2*_q.pData[3];			H[2] = -2*_q.pData[0];			H[3] =  2*_q.pData[1];
		H[4] =  2*_q.pData[1];		H[5] =  2*_q.pData[0];			H[6] =  2*_q.pData[3];			H[7] =  2*_q.pData[2];
		H[8] =  2*_q.pData[0];		H[9] = -2*_q.pData[1];			H[10] = -2*_q.pData[2];		H[11] =  2*_q.pData[3];
	}
	else if (flag == 1)
	{
		h[0] = _q.pData[0]*_q.pData[0] + _q.pData[1]*_q.pData[1] - _q.pData[2]*_q.pData[2] - _q.pData[3]*_q.pData[3];
		h[1] = 2*_q.pData[1]*_q.pData[2] - 2*_q.pData[0]*_q.pData[3]; 
		h[2] = 2*_q.pData[1]*_q.pData[3] + 2*_q.pData[0]*_q.pData[2]; 
		
		H[0] =  2*_q.pData[0];		H[1] =  2*_q.pData[1];			H[2] = -2*_q.pData[2];			H[3] = -2*_q.pData[3];
		H[4] = -2*_q.pData[3];		H[5] =  2*_q.pData[2];			H[6] =  2*_q.pData[1];			H[7] = -2*_q.pData[0];
		H[8] =  2*_q.pData[2];		H[9] =  2*_q.pData[3];			H[10] = 2*_q.pData[0];			H[11] = 2*_q.pData[1];
	}
	else
	{
		printf("Km45, neither acc or mag.\r\n");
	}
	
	// ����q
	arm_mat_sub_f32(&_z, &_h, &_v3);		// v3 = z-h  (3x1)
	arm_mat_mult_f32(&_K, &_v3, &_v4);	// v4 = K*(z-h)  (4x1)
	
	if (flag == 0)
	{
		_v4.pData[3] = 0;										// ���ٶȼ����������ı� q3
	}
	else if (flag ==1)
	{
		_v4.pData[1] = 0;
		_v4.pData[2] = 0;
	}
	arm_mat_add_f32(&_qi, &_v4, &_qo);		// qo = q_ + v4 (4x4)

	// ����P
	arm_mat_mult_f32(&_K, &_H, &_T);		// T = K*H  (4x4)
	arm_mat_sub_f32(&_I, &_T, &_M);			// M = I - K*H (4x4)
	arm_mat_mult_f32(&_M, &_P_, &_P);		// P = (I - K*H) * P_
}


/*
 * ��Ԫ��ģ����һ��
 */
static void NormalizeQuat(arm_matrix_instance_f32 *_q)
{
	float norm = sqrtf(_q->pData[0] * _q->pData[0]  + _q->pData[1] * _q->pData[1] + _q->pData[2]*_q->pData[2] + _q->pData[3]*_q->pData[3]);

	if (fabs(norm > 1e-8f))
		norm = 1.0f / norm;
	else
		norm = 1.0f;

	// ��һ����Ԫ��
	_q->pData[0] *= norm;
	_q->pData[1] *= norm;
	_q->pData[2] *= norm;
	_q->pData[3] *= norm;
}


/* 
 * ��Ԫ��������̬��
 * ��λ��deg
 */
static void Quat2EularAngleDeg(const arm_matrix_instance_f32 _q, struct EULAR_t *atti)
{
	atti->roll  = RAD2DEG * atan2f(2.0f*(_q.pData[2]*_q.pData[3]+_q.pData[0]*_q.pData[1]), _q.pData[0]*_q.pData[0]-_q.pData[1]*_q.pData[1]-_q.pData[2]*_q.pData[2]+_q.pData[3]*_q.pData[3]);
	atti->pitch = RAD2DEG * asinf(2.0f*(_q.pData[0]*_q.pData[2]-_q.pData[1]*_q.pData[3]) / (_q.pData[0]*_q.pData[0]+_q.pData[1]*_q.pData[1]+_q.pData[2]*_q.pData[2]+_q.pData[3]*_q.pData[3]));
	atti->yaw   = RAD2DEG * atan2f(2.0f*(_q.pData[1]*_q.pData[2]+_q.pData[0]*_q.pData[3]), (_q.pData[0]*_q.pData[0]+_q.pData[1]*_q.pData[1]-_q.pData[2]*_q.pData[2]-_q.pData[3]*_q.pData[3]));
}

/*
 * ��ȡ��ʼ��̬
 */
static uint8_t GetInitAtti(void)
{
	#define LEN 10
	#define N 5
	static float pitch[LEN] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
	static float roll[LEN] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
	static float yaw[LEN] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};

	static uint8_t k = 0;
	static uint8_t reach_max_len = 0;
	

	/* ����һ���ȴ�������Ч	*/
	
	if (fabs(eular_.roll) < 1e-8f  && fabs(eular_.pitch) < 1e-8f  && fabs(eular_.yaw) < 1e-8f)		// ������̬��ʼΪ0�����
		return 0;

	/* ������������������һ���ʼ����	*/
	if (reach_max_len == 0 && k < N*LEN)
	{
		roll[k % LEN] 	= eular_.roll;
		pitch[k % LEN] 	= eular_.pitch;
		yaw[k % LEN] 		= eular_.yaw;
		
		k ++;
		
		return 0;
	}
	
	/* ���������� */
	if (k >= N*LEN)
	{
		reach_max_len = 1;
	}
	
	
	/* �����������ݷ����ж��Ƿ��ʼ�����	*/
	k ++;
	k %= (N * LEN);
	
	roll	[k % LEN] 	= eular_.roll;
	pitch	[k % LEN] 	= eular_.pitch;
	yaw		[k % LEN] 	= eular_.yaw;
		
	
	if (k % N == 0)
	{
		if (Std(roll, LEN) < 0.02f  && Std(pitch, LEN) < 0.02f   &&   Std(yaw, LEN) < 0.1f)	// �����С
		{
			vTaskDelay(50);
			// ��ֵΪ��ʼŷ����
			eular0_.roll = Mean(roll, LEN);
			eular0_.pitch = Mean(pitch, LEN);
			eular0_.yaw = Mean(yaw, LEN);
			eular0_.valid = 1;
			
			// ŷ����ת��Ԫ��
			quat0_ = Eular2Quat(eular0_);
			quat0_.valid = 1;
			
			// ��Ԫ��ת��ת����
			Quat2Rbe(&quat0_, rot0_.R);
			rot0_.valid = 1;
			
			return 1;
		}
	}
	
	
	#undef LEN	
	
	return 0;
}

/*
 * 3ά�����ӻ���ϵת������ϵ
 */
struct XYZ_t VectorBody2Earth(const struct XYZ_t *b)
{
	struct XYZ_t e;
	
	e.x = rot_.R[0][0]*b->x + rot_.R[0][1]*b->y + rot_.R[0][2]*b->z;
	e.y = rot_.R[1][0]*b->x + rot_.R[1][1]*b->y + rot_.R[1][2]*b->z;
	e.z = rot_.R[2][0]*b->x + rot_.R[2][1]*b->y + rot_.R[2][2]*b->z;
	
	return e;
}


/*
 * ��ά��������ϵת��������ϵ
 */
struct XYZ_t VectorEarth2Body(const struct XYZ_t *e)
{
	struct XYZ_t b;
	
	b.x = rot_.R[0][0]*e->x + rot_.R[1][0]*e->y + rot_.R[2][0]*e->z;
	b.y = rot_.R[0][1]*e->x + rot_.R[1][1]*e->y + rot_.R[2][1]*e->z;
	b.z = rot_.R[0][2]*e->x + rot_.R[1][2]*e->y + rot_.R[2][2]*e->z;
	
	return b;
}




/*
 * ��ȡ��̬ŷ���ǽӿ�
 */
struct EULAR_t GetAttiEularApi(void)
{
	return eular_;
}

struct EULAR_t *PointAttiEularApi(void)
{
	return &eular_;
}


/*
 * ��ȡ��̬��Ԫ���ӿ�
 */
struct QUAT_t GetAttiQuatApi(void)
{
	return quat_;
}

struct QUAT_t *PointAttiQuatApi(void)
{
	return &quat_;
}



/*
 * ��ȡ��ת����ӿ�
 */
struct ROTATE_t *PointRotateApi(void)
{
	return &rot_;
}


/*
 * ��ȡ��ʼ����ת����
 */
struct ROTATE_t GetInitRotateApi(void)
{
	return rot0_;
}

struct ROTATE_t *PointInitRotateApi(void)
{
	return &rot0_;
}


/*
 * ��ȡ��ʼŷ����
 */
struct EULAR_t GetInitEularApi(void)
{
	return eular0_;
}

struct EULAR_t *PointInitEularApi(void)
{
	return &eular0_;
}



