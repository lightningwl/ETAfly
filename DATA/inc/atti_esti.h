#ifndef ATTI_ESTI_H
#define ATTI_ESTI_H


#include <arm_math.h>
#include "dof10.h"

void AttiEstiTask(void *arg);

static struct EULAR_t EularInit(void);
static struct QUAT_t QuatInit(void);


static void AccMean3(const struct XYZ_t *acc, struct XYZ_t *acc_mean);
static void GyroMean3(const struct XYZ_t *acc, struct XYZ_t *acc_mean);
static void Quat2EularAndCorrect(const arm_matrix_instance_f32 _q, struct EULAR_t *atti);

static void KmAttiEsti(const struct DOF9_t dof9, float dt);
static void NormalizeQuat(arm_matrix_instance_f32 *_q);
static void Quat2EularAngleDeg(const arm_matrix_instance_f32 _q, struct EULAR_t *atti);

static void Km12(const struct XYZ_t w, 
		const float dt, const arm_matrix_instance_f32 _q, 
		const arm_matrix_instance_f32 _P, 
		const arm_matrix_instance_f32 _Q ,
		arm_matrix_instance_f32 _q_, 
		arm_matrix_instance_f32 _P_);
		
static void Km3(const uint8_t flag, 
		const arm_matrix_instance_f32 _q, 
		const arm_matrix_instance_f32 _P_, 
		const arm_matrix_instance_f32 _R, 
		arm_matrix_instance_f32 _K);
		
static void Km45(const uint8_t flag, 
		const struct XYZ_t a, 
		const arm_matrix_instance_f32 _qi, 
		const arm_matrix_instance_f32 _q, 
		const arm_matrix_instance_f32 _P_, 
		const arm_matrix_instance_f32 _K, 
		arm_matrix_instance_f32 _qo, 
		arm_matrix_instance_f32 _P);

static uint8_t GetInitAtti(void);
		
struct EULAR_t GetAttiEularApi(void);
struct EULAR_t *PointAttiEularApi(void);
struct QUAT_t GetAttiQuatApi(void);
struct QUAT_t *PointAttiQuatApi(void);
struct ROTATE_t *PointRotateApi(void);
		
struct ROTATE_t GetInitRotateApi(void);
struct ROTATE_t *PointInitRotateApi(void);	

struct EULAR_t GetInitEularApi(void);		
		
struct XYZ_t VectorBody2Earth(const struct XYZ_t *b);
struct XYZ_t VectorEarth2Body(const struct XYZ_t *e);		
		
uint32_t GetAttiEstiTaskRunTimeApi(void);
		
#endif
