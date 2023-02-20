#ifndef DOF10_H
#define DOF10_H

#include "sys.h"   												  	  
#include "uavmath.h"
#include "mpu9250.h"



struct DOF6_t
{
	struct XYZ_t acc;
	struct XYZ_t gyro;
};

struct DOF9_t
{
	struct XYZ_t acc;
	struct XYZ_t gyro;
	struct XYZ_t mag;	
};


struct DOF10_t
{
	struct XYZ_t acc;
	struct XYZ_t gyro;
	struct XYZ_t mag;
	float alti;
};

void Dof10Task(void *arg);



struct DOF10_t GetDof10RawDataApi(void);
struct DOF10_t *PointDof10RawDataApi(void);

struct DOF9_t GetDof9CaliDataApi(void);
struct DOF9_t *PointDof9CaliDataApi(void);

struct DOF6_t GetDof6LpfDataApi(void);
struct DOF6_t *PointDof6LpfDataApi(void);


#endif

