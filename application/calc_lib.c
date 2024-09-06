#include "calc_lib.h"

//Î¢ÃëÑÓÊ±
void user_delay_us(uint16_t us)
{
	 for(; us > 0; us--)
	 {
		 for(uint8_t i = 50; i > 0; i--)
		 {
				;
		 }
	 }
}

//ºÁÃëÑÓÊ±
void user_delay_ms(uint16_t ms)
{
 for(; ms > 0; ms--)
 {
 user_delay_us(1000);
 }
}

//ÏÞ·ù
void abs_limit_fp(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//Ñ­»·ÏÞ·ù
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

