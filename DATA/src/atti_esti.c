/*
 * 卡尔曼滤波姿态估计
 */

#include "FreeRTOS.h"
#include "task.h"
#include "atti_esti.h"
#include "uavmath.h"
#include "dof10.h"
#include "delay.h"


struct EULAR_t eular_  	= {0, 0.0f, 0.0f, 0.0f};								// 当前欧拉角
struct QUAT_t quat_ 		= {0, 1.0f, 0.0f, 0.0f, 0.0f};					// 当前四元数
struct ROTATE_t rot_		= {0};																	// 旋转矩阵

struct EULAR_t eular0_  =	{0, 0.0f, 0.0f, 0.0f};								// 初始欧拉角
struct QUAT_t quat0_ 		= {0, 0.0f, 0.0f, 0.0f, 0.0f};					// 初始四元数
struct ROTATE_t rot0_		= {0};																	// 初始旋转矩阵


/*
 * 姿态估计任务
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
		vTaskDelayUntil(&t_last, 2);														// 定时调用, 500Hz
				
		dof9.acc = GetDof6LpfDataApi().acc;											// 获取加速度
		dof9.gyro = GetDof6LpfDataApi().gyro;										// 获取角速度
		Dof3Deg2Rad(&dof9.gyro);																// 角速度转化为弧度为单位
		
		if (count % 5 == 0)
			dof9.mag = GetDof9CaliDataApi().mag;									// 获取磁力计数据, 50Hz
		
		KmAttiEsti(dof9, dt);																		// 卡尔曼滤波姿态估计, 500Hz 
		
		if (eular0_.valid != 1)																	// 获取初始的姿态角
			GetInitAtti();
		
		count ++;
		count %= 100;
	}
}

/*
 * 卡尔曼滤波姿态估计
 * 输入：九轴传感器数据 dof9，加速度 m/s^2, 加速度 rad/s, 磁力计 IIC读取磁场强度*0.01
 * 此函数将更新全局变量 姿态四元数 quat_ 与 欧拉角 eular_
 */
void KmAttiEsti(const struct DOF9_t dof9, float dt)
{
	/* 卡尔曼滤波相关矩阵		*/
	static float32_t Q[16] = {1e-4f,0,0,0,   0,1e-4f,0,0,  0,0,1e-4f,0,  0,0,0,1e-4f};	// 过程噪声协方差矩阵
	static float32_t R[9] = {1,0,0,  0,1,0,    0,0,1};																	// 加速度计测量噪声协方差矩阵
	static float32_t R2[9] = {3,0,0,  0,3,0,    0,0,3};																	// 磁力计测量噪声协方差矩阵
	
	static float32_t q[4] = {1, 0, 0, 0};																								// 使用加速度计后验得到的协方差矩阵
	static float32_t q2[4] = {1, 0, 0, 0};																							// 使用磁力计后验得到的协方差矩阵
	static float32_t q_[4] = {1, 0, 0, 0};																							// 先验的q
	static float32_t P_[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};											// 先验的噪声协方差矩阵
	static float32_t P1[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}; 										// 通过加速度计后验得到的协方差矩阵、
	static float32_t P2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};											// 通过加磁力计后验得到的协方差矩阵、
	static float32_t K[12];																															// 卡尔曼增益
	static float32_t K2[12];																														// 卡尔曼增益
	
	
	/* 指向卡尔曼滤波相关矩阵的指针	*/
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

	/* 初始化卡尔曼滤波相关矩阵			*/
	
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
	else if (km_atti_times < 1000)										// 先信任加速度计和磁力计，收敛快
	{
		Q[0] = 1e-3f;
	  Q[5] = 1e-3f;
		Q[10] = 1e-3f;
		Q[15] = 1e-3f;
		km_atti_times ++;
	}
	else																							// 后信任陀螺仪，准确
	{
		Q[0] = 1e-8f;
	  Q[5] = 1e-8f;
		Q[10] = 1e-8f;
		Q[15] = 1e-8f;		
	}
	
	/* 卡尔曼滤波过程	*/
	Km12(dof9.gyro, dt, _q2, _P2, _Q, _q_, _P_);			// 卡尔曼公式1,2 根据陀螺仪预测
	Km3(0, _q2, _P_, _R, _K);													// 卡尔曼公式3，计算卡尔曼增益
	Km45(0, dof9.acc, _q_, _q2, _P_, _K, _q, _P1);		// 卡尔曼公式4,5 根据加速度修正
	Km3(1, _q2, _P_, _R2, _K2);												// 卡尔曼公式3，计算卡尔曼增益
	Km45(1, dof9.mag, _q, _q2, _P1, _K2, _q2, _P2);		// 卡尔曼公式4，5 根据磁力计修正
	
	NormalizeQuat(&_q2);															// 四元数归一化
	
	/* 赋值给全局变量 */
	quat_.q0 = _q2.pData[0];													// 赋值给全局变量
	quat_.q1 = _q2.pData[1];
	quat_.q2 = _q2.pData[2];
	quat_.q3 = _q2.pData[3];
	
	Quat2EularAngleDeg(_q2, &eular_);									// 转化为欧拉角，赋值给全局变量
	Quat2Rbe(&quat_, rot_.R);														// 转化为旋转矩阵
}

/* 
 * 卡尔曼公式1,2--计算先验的q_和P_
 * 输入：机体角速度 w (rad/s), (3x1)
 * 			时间步长 dt (s)
 *			上一次后验的四元数 q (4x1)
 *			上一次后验的协方差矩阵 P (4x4)
 *			过程噪声协方差矩阵Q (4x4)
 * 输出：先验状态，四元数 q_ (4x1)
 *			先验协方差矩阵 P_  (4x4)
 */
static void Km12(const struct XYZ_t w, const float dt, const arm_matrix_instance_f32 _q, const arm_matrix_instance_f32 _P, const arm_matrix_instance_f32 _Q ,arm_matrix_instance_f32 _q_, arm_matrix_instance_f32 _P_)
{
	float32_t A[16];
	arm_matrix_instance_f32 _A;
	
	float32_t A_T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// 矩阵A的转置
	arm_matrix_instance_f32 _A_T;

	float32_t T[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// 临时定义中间变量
	arm_matrix_instance_f32 _T;
	
	float32_t T2[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};		// 临时定义中间变量
	arm_matrix_instance_f32 _T2;
	
	float temp;
	
	
	temp = dt / 2;

	A[0] = 1.0f; 			A[1] = -w.x*temp; 	A[2] = -w.y*temp; 	A[3] = -w.z*temp;
	A[4] = w.x*temp;	A[5] = 1.0f;				A[6] = w.z*temp;		A[7] = -w.y*temp;
	A[8] = w.y*temp;	A[9] = -w.z*temp;		A[10] = 1.0f;				A[11] = w.x*temp;
	A[12] = w.z*temp;	A[13]= w.y*temp;		A[14] = -w.x*temp;	A[15] = 1.0f;
	
	arm_mat_init_f32(&_A, 4, 4, A);				// 转移矩阵A (4x4)
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
 * 卡尔曼公式3--计算卡尔曼增益
 * 输入：上一次估计四元数 q (4x1)
 *			先验的噪声协方差矩阵 P_ (4x4)
 *			测量噪声协方差矩阵 R (3x3)
 * 输出：卡尔曼增益 K (4x3)
 */
static void Km3(const uint8_t flag, const arm_matrix_instance_f32 _q, const arm_matrix_instance_f32 _P_, const arm_matrix_instance_f32 _R, arm_matrix_instance_f32 _K)
{
	float32_t H[12];	// (3x4)
	float32_t H_T[12];// (4x3)
	float32_t T[12];// (3x4)
	float32_t T1[9];// (3x3)
	float32_t M[9];			// (3x3)
	float32_t M_[9];		// M的逆, (3x3)
	
	arm_matrix_instance_f32 _H;
	arm_matrix_instance_f32 _H_T;
	arm_matrix_instance_f32 _T;
	arm_matrix_instance_f32 _T1;
	arm_matrix_instance_f32 _M;
	arm_matrix_instance_f32 _M_;

	// 观测矩阵
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
	
	// 矩阵初始化
	arm_mat_init_f32(&_H, 3, 4, H);				// 观测矩阵H (3x4)
	arm_mat_init_f32(&_H_T, 4, 3, H_T);
	arm_mat_init_f32(&_T, 3, 4, T);				// T (3x4)
	arm_mat_init_f32(&_M, 3, 3, M);				// M (3x3)
	arm_mat_init_f32(&_M_, 3, 3, M_);			// M_(3x3)
	arm_mat_init_f32(&_T1, 3, 3, T1);
	
	// 卡尔曼增益
	arm_mat_mult_f32(&_H, &_P_, &_T);			// T = H*P_		(3x4)
	arm_mat_trans_f32(&_H, &_H_T);				// H' (4x3)
	arm_mat_mult_f32(&_T, &_H_T, &_T1);		// T1 = H*P_*H' (3x3)
	
	arm_mat_add_f32(&_T1, &_R, &_M);			// M = H*P_*H' + R				 (3x3)
	arm_mat_inverse_f32(&_M, &_M_);				// M_ = inv(H*P_*H' + R)		 (3x3)
	arm_mat_init_f32(&_T, 4, 3, T);				// 将T初始化为 4x3 矩阵
	arm_mat_mult_f32(&_P_, &_H_T, &_T);		// T = P_ * H' (4x3)
	arm_mat_mult_f32(&_T, &_M_, &_K);			// K = P_ * H' * inv(H1 * P_ * H' + R);  (4x3)
}


/*
 * 卡尔曼公式4,5
 * 输入：观测的状态 a (加速度单位为 N/s^2） (3x1)
 *			先验的四元数 qi_ (4x1)
 *      上一次估计的 q
 *			先验的噪声协方差矩阵 P_
 *			卡尔曼增益 K (4x3)
 * 输出：后验的四元数 qo (4x1)
 *			后验的噪声协方差矩阵 P (4x4)
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
	float norm 	; 			// 模长
	
	float32_t z[3];			// 观测的向量
	float32_t h[3];			// 地球系向量在机体系下的分量 (3x1)
	float32_t v3[3];		// 临时向量 (3x1)
	float32_t v4[4];		// 临时向量（4x1）
	float32_t I[16]= {1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1};		// 单位阵（4x4）
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
	
	// 获取测量值
	z[0] = a.x;
	z[1] = a.y;
	z[2] = a.z;
	
	// 矩阵初始化
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
	
	// 理论值
	if (flag == 0)
	{
		h[0] = 2*_q.pData[1]*_q.pData[3] - 2*_q.pData[0]*_q.pData[2];
		h[1] = 2*_q.pData[0]*_q.pData[1] + 2*_q.pData[2]*_q.pData[3];
		h[2] = _q.pData[0]*_q.pData[0] - _q.pData[1]*_q.pData[1] - _q.pData[2]*_q.pData[2] + _q.pData[3]*_q.pData[3]; 
	
		// 观测矩阵
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
	
	// 修正q
	arm_mat_sub_f32(&_z, &_h, &_v3);		// v3 = z-h  (3x1)
	arm_mat_mult_f32(&_K, &_v3, &_v4);	// v4 = K*(z-h)  (4x1)
	
	if (flag == 0)
	{
		_v4.pData[3] = 0;										// 加速度计修正，不改变 q3
	}
	else if (flag ==1)
	{
		_v4.pData[1] = 0;
		_v4.pData[2] = 0;
	}
	arm_mat_add_f32(&_qi, &_v4, &_qo);		// qo = q_ + v4 (4x4)

	// 修正P
	arm_mat_mult_f32(&_K, &_H, &_T);		// T = K*H  (4x4)
	arm_mat_sub_f32(&_I, &_T, &_M);			// M = I - K*H (4x4)
	arm_mat_mult_f32(&_M, &_P_, &_P);		// P = (I - K*H) * P_
}


/*
 * 四元数模长归一化
 */
static void NormalizeQuat(arm_matrix_instance_f32 *_q)
{
	float norm = sqrtf(_q->pData[0] * _q->pData[0]  + _q->pData[1] * _q->pData[1] + _q->pData[2]*_q->pData[2] + _q->pData[3]*_q->pData[3]);

	if (fabs(norm > 1e-8f))
		norm = 1.0f / norm;
	else
		norm = 1.0f;

	// 归一化四元数
	_q->pData[0] *= norm;
	_q->pData[1] *= norm;
	_q->pData[2] *= norm;
	_q->pData[3] *= norm;
}


/* 
 * 四元数计算姿态角
 * 单位：deg
 */
static void Quat2EularAngleDeg(const arm_matrix_instance_f32 _q, struct EULAR_t *atti)
{
	atti->roll  = RAD2DEG * atan2f(2.0f*(_q.pData[2]*_q.pData[3]+_q.pData[0]*_q.pData[1]), _q.pData[0]*_q.pData[0]-_q.pData[1]*_q.pData[1]-_q.pData[2]*_q.pData[2]+_q.pData[3]*_q.pData[3]);
	atti->pitch = RAD2DEG * asinf(2.0f*(_q.pData[0]*_q.pData[2]-_q.pData[1]*_q.pData[3]) / (_q.pData[0]*_q.pData[0]+_q.pData[1]*_q.pData[1]+_q.pData[2]*_q.pData[2]+_q.pData[3]*_q.pData[3]));
	atti->yaw   = RAD2DEG * atan2f(2.0f*(_q.pData[1]*_q.pData[2]+_q.pData[0]*_q.pData[3]), (_q.pData[0]*_q.pData[0]+_q.pData[1]*_q.pData[1]-_q.pData[2]*_q.pData[2]-_q.pData[3]*_q.pData[3]));
}

/*
 * 获取初始姿态
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
	

	/* 步骤一，等待数据有效	*/
	
	if (fabs(eular_.roll) < 1e-8f  && fabs(eular_.pitch) < 1e-8f  && fabs(eular_.yaw) < 1e-8f)		// 避免姿态初始为0的情况
		return 0;

	/* 步骤二，先向数组填充一组初始数据	*/
	if (reach_max_len == 0 && k < N*LEN)
	{
		roll[k % LEN] 	= eular_.roll;
		pitch[k % LEN] 	= eular_.pitch;
		yaw[k % LEN] 		= eular_.yaw;
		
		k ++;
		
		return 0;
	}
	
	/* 填充数据完毕 */
	if (k >= N*LEN)
	{
		reach_max_len = 1;
	}
	
	
	/* 步骤三，根据方差判断是否初始化完毕	*/
	k ++;
	k %= (N * LEN);
	
	roll	[k % LEN] 	= eular_.roll;
	pitch	[k % LEN] 	= eular_.pitch;
	yaw		[k % LEN] 	= eular_.yaw;
		
	
	if (k % N == 0)
	{
		if (Std(roll, LEN) < 0.02f  && Std(pitch, LEN) < 0.02f   &&   Std(yaw, LEN) < 0.1f)	// 方差很小
		{
			vTaskDelay(50);
			// 均值为初始欧拉角
			eular0_.roll = Mean(roll, LEN);
			eular0_.pitch = Mean(pitch, LEN);
			eular0_.yaw = Mean(yaw, LEN);
			eular0_.valid = 1;
			
			// 欧拉角转四元数
			quat0_ = Eular2Quat(eular0_);
			quat0_.valid = 1;
			
			// 四元数转旋转矩阵
			Quat2Rbe(&quat0_, rot0_.R);
			rot0_.valid = 1;
			
			return 1;
		}
	}
	
	
	#undef LEN	
	
	return 0;
}

/*
 * 3维向量从机体系转到地球系
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
 * 三维向量地球系转换到机体系
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
 * 获取姿态欧拉角接口
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
 * 获取姿态四元数接口
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
 * 获取旋转矩阵接口
 */
struct ROTATE_t *PointRotateApi(void)
{
	return &rot_;
}


/*
 * 获取初始的旋转矩阵
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
 * 获取初始欧拉角
 */
struct EULAR_t GetInitEularApi(void)
{
	return eular0_;
}

struct EULAR_t *PointInitEularApi(void)
{
	return &eular0_;
}



