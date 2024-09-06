/**
  ****************************(C) COPYRIGHT 2021 QUT****************************
  * @file       motor_control.c/h
  * @brief      3508电机初始化、速度环、位置环；6020电机初始化、位置环（相对角度）
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-23-2021     QUT              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 QUT****************************
  */
#include "motor_control.h"
#include "CAN_receive.h"
#include "pid.h"
#include "calc_lib.h"


//3508电机初始化ID1~4
//mode	1:速度环
//			2:位置环
void M3508_motor_init(int mode);
//3508速度环ID1~4
void M3508_velocity_loop(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4);
//3508位置环ID1~4
void M3508_pos_loop(fp32 angle1,fp32 angle2,fp32 angle3,fp32 angle4);
//6020电机初始化ID1
void GM6020_position_loop_init(void);
//6020位置环ID1（相对角度）,该方法6020电机仅能转小于180的角
void GM6020_pos_loop(int32_t ecd);


const motor_measure_t *motor_data[4];//3508电机数据
pid_type_def motor_pid[4];//3508速度环pid结构体
pid_type_def angle_pid[4];//3508位置环pid结构体
//位置环串级pid参数（10ms间隔）
const fp32 speed_PID[3]={5.0,0.3,0.0};	//P,I,D(速度环)
const fp32 angle_PID[3]={25.0,0.0,1.5};	//P,I,D(角度环)
//速度环pid参数
const fp32 PID[3]={5.0,0.01,0.0};

const motor_measure_t *GM6020_data;//6020电机数据
gimbal_PID_t gimbal_motor_relative_angle_pid;//6020相对角度位置环pid结构体
pid_type_def gimbal_motor_gyro_pid;//6020相对角度速度环pid结构体
gimbal_pid_control_t GM6020_pos_loop_control;//6020位置控制结构体
const fp32 GM6020_PID[3]={3600.0,20.0,0};//6020云台PID(速度环)

//*****3508*****//
//3508电机初始化ID1~4
//mode	1:速度环
//			2:位置环
void M3508_motor_init(int mode)
{
	//获取3508电机数据
	motor_data[0] = get_chassis_motor_measure_point(0);
	motor_data[1] = get_chassis_motor_measure_point(1);
	motor_data[2] = get_chassis_motor_measure_point(2);
	motor_data[3] = get_chassis_motor_measure_point(3);
	
	switch(mode)
	{
		//速度环初始化
		case 1:
						for(int i=0;i<4;i++)
						{
							PID_init(&motor_pid[i],PID_POSITION,PID,16000,2000);		//速度环pid初始化
						}break;
		//位置环初始化
		case 2:
						for(int i=0;i<4;i++)
						{
							PID_init(&motor_pid[i],PID_POSITION,speed_PID,16000,2000);		//速度环pid初始化
							PID_init(&angle_pid[i],PID_POSITION,angle_PID,1000,200);		//位置环pid初始化
						}break;
	}

}

//3508速度环
void M3508_velocity_loop(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4)
{
	int32_t speed[4];
	int32_t delta_speed[4];
	int i;
	
	speed[0] = speed1;
	speed[1] = speed2;
	speed[2] = speed3;
	speed[3] = speed4;
	
	for(i=0;i<4;i++)
	{
		delta_speed[i] = PID_calc(&motor_pid[i],motor_data[i]->speed_rpm,speed[i]);
	}
	
	CAN_cmd_chassis(delta_speed[0],delta_speed[1],delta_speed[2],delta_speed[3]);
}

//3508位置环
void M3508_pos_loop(fp32 angle1,fp32 angle2,fp32 angle3,fp32 angle4)
{
	fp32 angle[4];
	fp32 delta_angle[4];
	int32_t delta_speed[4];
	int i;
	
	angle[0] = angle1;
	angle[1] = angle2;
	angle[2] = angle3;
	angle[3] = angle4;
	
	//外环角度环
	for(i=0;i<4;i++)
	{
		delta_angle[i] = PID_calc(&angle_pid[i],motor_data[i]->total_angle,angle[i]);
	}
	//内环速度环
	for(i=0;i<4;i++)
	{
		delta_speed[i] = PID_calc(&motor_pid[i],motor_data[i]->speed_rpm,delta_angle[i]);
	}
	
	CAN_cmd_chassis(delta_speed[0],delta_speed[1],delta_speed[2],delta_speed[3]);
}


//*****6020*****//
//6020电机初始化ID1
void GM6020_position_loop_init(void)
{
	gimbal_PID_init(&GM6020_pos_loop_control.gimbal_motor_relative_angle_pid,10.0f,0.0f,8.0f,5.0f,0.0f);		//角度环pid初始化
	PID_init(&GM6020_pos_loop_control.gimbal_motor_gyro_pid,PID_POSITION,GM6020_PID,30000.0,5000.0);		//速度环pid初始化
	
	GM6020_data=get_yaw_gimbal_motor_measure_point();
}

//6020位置环ID1（相对角度）
//该方法6020电机仅能转小于180的角
//ecd 编码器值(0-8191)
void GM6020_pos_loop(int32_t ecd)
{
	//给定目标位置
	GM6020_pos_loop_control.angel_set=rad(ecd);
	//计算相对角度(-PI,PI)
	GM6020_pos_loop_control.angel_get=motor_ecd_to_angle_change(GM6020_data->ecd,GM6020_pos_loop_control.angel_set);
	//计算角速度gyro (set-get)
	GM6020_pos_loop_control.gyro_get=rad(GM6020_pos_loop_control.angel_set-GM6020_pos_loop_control.angel_get);
	//角度环，速度环串级pid调试
  GM6020_pos_loop_control.gyro_set = gimbal_PID_calc(&GM6020_pos_loop_control.gimbal_motor_relative_angle_pid,GM6020_pos_loop_control.angel_get, GM6020_pos_loop_control.angel_set,GM6020_pos_loop_control.gyro_get);
  GM6020_pos_loop_control.current_set = PID_calc(&GM6020_pos_loop_control.gimbal_motor_gyro_pid,GM6020_pos_loop_control.gyro_get, GM6020_pos_loop_control.gyro_set);
	
  //控制值赋值
  GM6020_pos_loop_control.current_given = (int16_t)(GM6020_pos_loop_control.current_set);
	CAN_cmd_gimbal(GM6020_pos_loop_control.current_given,0,0,0);
	
}





void gimbal_PID_init(gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
//    if (pid == NULL)
//    {
//        return;
//    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

fp32 gimbal_PID_calc(gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
//    if (pid == NULL)
//    {
//        return 0.0f;
//    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit_fp(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit_fp(&pid->out, pid->max_out);
    return pid->out;
}

//相对角度计算函数（-4096,4095）
fp32 motor_ecd_to_angle_change(uint32_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 4095)
    {
      while(relative_ecd>4095)
			{
				relative_ecd -= 8191;
			}
    }
    else if (relative_ecd < -4096)
    { 
			while(relative_ecd<-4096)
			{
				relative_ecd += 8191;
			}
    }
		return relative_ecd * MOTOR_ECD_TO_RAD;
}


