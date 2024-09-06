#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include "struct_typedef.h"
#include "pid.h"

//typedef struct
//{
//	const fp32 speed_PID[3];//P,I,D
//	const fp32 angle_PID;
//}

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} gimbal_PID_t;

typedef struct
{
	gimbal_PID_t gimbal_motor_relative_angle_pid;
	pid_type_def gimbal_motor_gyro_pid;
	fp32 angel_set;
	fp32 angel_get;
	fp32 gyro_set;
	fp32 gyro_get;
	fp32 current_set;
	fp32 current_given;
}gimbal_pid_control_t;


#define FILTER_BUF_LEN		5

//#define MOTOR_ECD_TO_ANGLE 0.00231311936
#define MOTOR_ECD_TO_ANGLE  (360.0 / 8191.0 / (3591.0/187.0))
#define MOTOR_ECD_TO_RAD 0.000766990394f
#define rad_format(Ang) loop_fp32_constrain((Ang), (-4096*MOTOR_ECD_TO_RAD), (4095*MOTOR_ECD_TO_RAD))
#define rad(code) ((code)*MOTOR_ECD_TO_RAD)


void M3508_motor_init(int mode);
void M3508_pos_loop(fp32 angle1,fp32 angle2,fp32 angle3,fp32 angle4);
void GM6020_position_loop_init(void);
void GM6020_pos_loop(int32_t ecd);
void M3508_velocity_loop(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4);
fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
fp32 motor_ecd_to_angle_change(uint32_t ecd, uint16_t offset_ecd);
void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);

#endif 

