#include "imu_temp_control_task.h"
#include "BMI088driver.h"
#include "main.h"
#include "pid.h"
#include "vofa.h"
//#include "bsp_imu_pwm.h"

//#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm¸ø¶¨

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4600.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4600.0f //max iout of temperature control PID 


extern SPI_HandleTypeDef hspi1;


fp32 gyro[3], accel[3], temp;

//kp, ki,kd three params
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;

float temp_data[4];
void imu_temp_control_init(void )
{
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

}

void imu_temp_control_task(void )
{

    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

        BMI088_read(gyro, accel, &temp);
        PID_calc(&imu_temp_pid, temp, 40.0f);
    
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
	   temp_data[2]=imu_temp_pid.set;
	   temp_data[1]=temp;
	   vofa_tx_main(temp_data);
	   
	   TIM10->CCR1 = imu_temp_pid.out ;
	   
	   HAL_Delay (5);


    }



