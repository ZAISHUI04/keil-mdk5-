#ifndef _CALC_LIB_H_
#define _CALC_LIB_H_

#include "struct_typedef.h"

void user_delay_ms(uint16_t ms);
void abs_limit_fp(fp32 *num, fp32 Limit);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
#endif

