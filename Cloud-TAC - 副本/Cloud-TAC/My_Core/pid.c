#include "pid.h"




void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid,         //PID结构体地址
    uint32_t mode,      //PID模式
    float maxout,    //PID输出最大值
    float intergral_limit, //PID积分限幅
    float 	kp,         //比例系数
    float 	ki,         //积分系数
    float 	kd,        //微分系数
		float Max_err, 
		float Deadband)  
{   
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
	  pid->max_err=Max_err;
	  pid->deadband=Deadband;    
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}
/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set){
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )  //设置量程 如果超量程 则失能   
		return 0;
	if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)     //设置死区   与上述参数相同 可单独初始化
		return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式PID
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式PID
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);       
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;	
}
/*pid总体初始化-----------------------------------------------------------------*/   //主要还是用这个函数
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    float maxout,
    float intergral_limit,   
    float 	kp, 
    float 	ki, 
    float 	kd,
		float Max_err,
		float Deadband)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;    //获取函数地址  （指针函数）
    pid->f_pid_reset = pid_reset;		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd,Max_err,Deadband);   //初始化函数参数	
}



















 






