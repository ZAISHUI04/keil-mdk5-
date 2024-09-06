#include "chassis.h"
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "BMI088driver.h"
#include "motor_control.h"
pid_type_def motor_pid_a[4];//���pid�ṹ��
pid_type_def motor_2006_pid;
const motor_measure_t *motor_data_a[4];//3508�������
const motor_measure_t *motor_2006;
const fp32 PID_2006[3]={10,0.01,0};
const fp32 PID_CHASSI[3]={5,0.01,0};	//P,I,D
fp32 chassi_gyro[3],chassi_accel[3],chassi_temp;	//������
extern RC_ctrl_t rc_ctrl;




int16_t motor_2006_set;//2006���Ŀ���ٶ�
int16_t vx_set,vy_set,vw_set;//x,y,wĿ���ٶ�
int16_t wheel_speed[4];//�ֽ��ٶ�
int16_t abc=0;

//���̳�ʼ��
void chassis_init(void)
{
	can_filter_init();//can��������ʼ��
	remote_control_init();
     GM6020_position_loop_init();


//	while(BMI088_init())		//�����ǳ�ʼ��
//	{
//		;
//	}
	for(int i=0;i<4;i++)		
	{
		PID_init(&motor_pid_a[i],PID_POSITION,PID_CHASSI,10000,2000);//pid��ʼ��
		motor_data_a[i] = get_chassis_motor_measure_point(i);//��ȡ�������ָ��
	}
}

void RC_chassis_mode(void)
{

	int16_t motor_set_speed[4];//���Ŀ���ٶ�
	float mul=0;//���ű���,�ٷֱ�
	//������ת |����  *  �������� |ǰ��
	vx_set = rc_ctrl.ch[1];
	vy_set = rc_ctrl.ch[0];
	vw_set = rc_ctrl.ch[3];
     mul = (float)(vy_set + 323) / 608 ;
	
	int16_t gyro_z=0;//�����ǵ�����,����������ת
//    abc=(float )(rc_ctrl.sw[1]-306)/1388*180*25;
	
//	gyro_z = gyro[2]*800;
	if(vw_set!=0 || (vx_set==0&&vy_set==0)) gyro_z=0;			//�������תָ�����̾�ֹ�������ǲ�����


	wheel_speed[0] = (vx_set+vy_set+vw_set)*mul + gyro_z;//��ǰ
	wheel_speed[1] = (-vx_set+vy_set+vw_set)*mul + gyro_z;//��ǰ
	wheel_speed[2] = (-vx_set-vy_set+vw_set)*mul + gyro_z;//�Һ�
	wheel_speed[3] = (vx_set-vy_set+vw_set)*mul + gyro_z;//���
	
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
//	can_filter_init();//can��������ʼ��
//	remote_control_init();
//	PID_init (&motor_2006_pid,PID_POSITION,PID_2006,16384,5000);
//	motor_2006=get_trigger_motor_measure_point();
//}

//void motor_2006_control(void)
//{
//	int16_t motor_2006_set;//2006���Ŀ���ٶ�
//	int16_t motor_2006_set_speed;//�����ٶ�
//	motor_2006_set=rc_ctrl.sw[3];
//	motor_2006_set_speed=PID_calc(&motor_2006_pid,motor_2006->speed_rpm,motor_2006_set);
//	CAN_cmd_gimbal(0, 0, motor_2006_set_speed,  0);//����6020һ��Ҫ�ģ�����Ĭ����������ٶ�д��
//	HAL_Delay (2);
//	
//}


