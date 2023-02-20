#ifndef HEIGHT_ESTI_H
#define HEIGHT_ESTI_H


#include "sys.h"
#include <arm_math.h>
#include "dof10.h"

static void KmPosEsti(float dt, 	// 时间间隔
								float a,		// 地球系下加速度 (m/s^2)
								float p,		// 气压计或GPS测量位移 (m)
								const float R,
								const arm_matrix_instance_f32 _Q,
								arm_matrix_instance_f32 _P,
								arm_matrix_instance_f32 _x
								);

static void KmPosVelEsti(float dt, 	// 时间间隔
								float a,		// 地球系下加速度 (m/s^2)
							  float v,    // 地球系下速度(m/s)
								float p,		// 气压计或GPS测量位移 (m)
								const arm_matrix_instance_f32 _R,
								const arm_matrix_instance_f32 _Q,
								arm_matrix_instance_f32 _P,
								arm_matrix_instance_f32 _x
								);

static void PosxEsti(const float dt, const float acc, const float vel,  const float pos, 
								const float R, const float Q, 
								float *pos_esti, float *vel_esti);

static void PosyEsti(const float dt, const float acc, const float vel,  const float pos, 
								const float R, const float Q, 
								float *pos_esti, float *vel_esti);	
	
static void PoszEsti(const float dt, const float acc, const float pos, 
	const float R, const float Q, 
	float *pos_esti, float *vel_esti);
	
static struct XYZ_t GetInitPos(void);
static void AcceMean10(const struct XYZ_t *acc, struct XYZ_t *acc_mean);
struct XYZ_t Ode2Earth(const struct XYZ_t *ode);
								
					
static struct XYZ_t GetAccBias(float dt, float p, float p_km);	
	
static struct XYZ_t AccBody2Earth(const struct XYZ_t *b);
struct XYZ_t *PointEarthAccApi(void);
void PosEstiTask(void *arg);
								
float GetPzBaroApi(void);
float GetPzFuseApi(void);
float GetVzFuseApi(void);				
float _GetAcceNoMean(void);			

struct XYZ_t GetRawPosApi(void);
struct XYZ_t *PointRawPosApi(void);
struct XYZ_t GetFusePosApi(void);
struct XYZ_t *PointFusePosApi(void);
struct XYZ_t GetFuseVelApi(void);
struct XYZ_t *PointFuseVelApi(void);

#endif
