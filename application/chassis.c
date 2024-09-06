#include "chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include "motor_control.h"
pid_type_def motor_pid_a[4];//电机pid结构体
pid_type_def motor_2006_pid;
const motor_measure_t *motor_data_a[4];//3508电机数据
const motor_measure_t *motor_2006;
const fp32 PID_2006[3]={10,0.01,0};
const fp32 PID_CHASSI[3]={5,0.01,0};	//P,I,D
fp32 chassi_gyro[3],chassi_accel[3],chassi_temp;	//陀螺仪
extern RC_ctrl_t rc_ctrl;




int16_t motor_2006_set;//2006电机目标速度
int16_t vx_set,vy_set,vw_set;//x,y,w目标速度
int16_t wheel_speed[4];//分解速度
int16_t abc=0;

//底盘初始化
void chassis_init(void)
{
	can_filter_init();//can过滤器初始化
	remote_control_init();
     GM6020_position_loop_init();


//	while(BMI088_init())		//陀螺仪初始化
//	{
//		;
//	}
	for(int i=0;i<4;i++)		
	{
		PID_init(&motor_pid_a[i],PID_POSITION,PID_CHASSI,10000,2000);//pid初始化
		motor_data_a[i] = get_chassis_motor_measure_point(i);//获取电机数据指针
	}
}

void RC_chassis_mode(void)
{

	int16_t motor_set_speed[4];//电机目标速度
	float mul=0;//油门倍率,百分比
	//——旋转 |油门  *  ——左右 |前后
	vx_set = rc_ctrl.ch[1];
	vy_set = rc_ctrl.ch[0];
	vw_set = rc_ctrl.ch[3];
     mul = (float)(vy_set + 323) / 608 ;
	
	int16_t gyro_z=0;//陀螺仪调整量,纠正意外旋转
//    abc=(float )(rc_ctrl.sw[1]-306)/1388*180*25;
	
//	gyro_z = gyro[2]*800;
	if(vw_set!=0 || (vx_set==0&&vy_set==0)) gyro_z=0;			//如果有旋转指令或底盘静止，陀螺仪不调整


	wheel_speed[0] = (vx_set+vy_set+vw_set)*mul + gyro_z;//左前
	wheel_speed[1] = (-vx_set+vy_set+vw_set)*mul + gyro_z;//右前
	wheel_speed[2] = (-vx_set-vy_set+vw_set)*mul + gyro_z;//右后
	wheel_speed[3] = (vx_set-vy_set+vw_set)*mul + gyro_z;//左后
	
	for(int i=0;i<4;i++)
	{
		motor_set_speed[i] = PID_calc(&motor_pid_a[i], motor_data_a[i]->speed_rpm, wheel_speed[i]); 
	}
	
	GM6020_pos_loop(0);
	CAN_cmd_chassis(motor_set_speed[0],motor_set_speed[1],motor_set_speed[2],motor_set_speed[3]);
	
	HAL_Delay (5);
	
}

//void motor_2006_control_init(void)
//{
//	can_filter_init();//can过滤器初始化
//	remote_control_init();
//	PID_init (&motor_2006_pid,PID_POSITION,PID_2006,16384,5000);
//	motor_2006=get_trigger_motor_measure_point();
//}

//void motor_2006_control(void)
//{
//	int16_t motor_2006_set;//2006电机目标速度
//	int16_t motor_2006_set_speed;//计算速度
//	motor_2006_set=rc_ctrl.sw[3];
//	motor_2006_set_speed=PID_calc(&motor_2006_pid,motor_2006->speed_rpm,motor_2006_set);
//	CAN_cmd_gimbal(0, 0, motor_2006_set_speed,  0);//后有6020一定要改，这里默认其他电机速度写零
//	HAL_Delay (2);
//	
//}


