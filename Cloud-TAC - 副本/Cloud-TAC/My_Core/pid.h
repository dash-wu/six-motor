#ifndef __PID_H
#define __PID_H

#include "stdio.h"
#include "stm32h7xx.h"
#include <math.h>
#define ABS(x)		((x>0)? (x): (-x))      //È¡¾ø¶ÔÖµ

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;   
    float set[3];				
    float get[3];				
    float err[3];				   
    float pout;							
    float iout;							
    float dout;						 
    float pos_out;						
    float last_pos_out;				
    float delta_u;						
    float delta_out;					
    float last_delta_out;   
	  float max_err;
	  float deadband;				
    uint32_t pid_mode;
    float MaxOutput;				
    float IntegralLimit;		   
    void (*f_param_init)(struct __pid_t *pid,  
                    uint32_t pid_mode,
                    float maxOutput,
                    float integralLimit,
                    float p,
                    float i,
                    float d,
										float Max_err,
		                float Deadband);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);	
}pid_t;

void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    float maxout,
    float intergral_limit,   
    float 	kp, 
    float 	ki, 
    float 	kd,
		float Max_err,
		float Deadband);    
    float pid_calc(pid_t* pid, float get, float set);

#endif
