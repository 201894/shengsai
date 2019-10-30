
/** @file pid.c
 *  @version 1.0
 *  @date Jan 2019
 *
 *  @brief pid parameter initialization, position and delta pid calculate
 *
 */	
#include "pid.h" 
#include <math.h>
#include <string.h>
#include <stdio.h>

PID_Typedef pid_yaw_out       ={0};//6020
PID_Typedef pid_yaw_in         ={0};
PID_Typedef pid_fric       ={0};//6020
PID_Typedef pid_sick_out       ={0};//6020

/**
 * @brief Absolute PID algorithm
 * @param PID_AbsoluteType
 * @return None
 */
void pid_ast(PID_Typedef *pid,float target,float input,int flag)
{
	pid->errNow = (target - input);
	if(flag)
	{
	 if(pid->errNow <= -180)
	 {
		pid->errNow += 360;
	 }
	 if(pid->errNow >= 180)
	 {
		pid->errNow -= 360;
	 }	
    }	
	pid->errP = pid->errNow;
	if(pid->errIPoint == 0){
		pid->errI += pid->errNow;
	}else{
		if(fabs(pid->errNow) < fabs(pid->errIPoint)){
			pid->errI += (pid->errIPoint - fabs(pid->errNow))/pid->errIPoint*pid->errNow;
		}
	}
	if(pid->errILim != 0){
		if(pid->errI >  pid->errILim)      pid->errI =  pid->errILim;
		else if(pid->errI < -pid->errILim) pid->errI = -pid->errILim;
	}
	pid->errD = pid->errNow-pid->errOld;
	if(fabs(pid->errNow) < fabs(pid->errDPoint)){
		pid->errD *= 0.1;
	}
	pid->errOld =pid->errNow;
	pid->ctrOut = pid->kp*pid->errP + pid->ki * pid->errI + pid->kd*pid->errD;
	
	if(pid->MaxOut != 0){
		if(pid->ctrOut >  pid->MaxOut) pid->ctrOut =  pid->MaxOut;
		if(pid->ctrOut < -pid->MaxOut) pid->ctrOut = -pid->MaxOut;
	}
}

/**
 * @brief Increment PID algorithm
 * @param PID_IncrementType
 * @return None
 */
void PID_IncrementMode(PID_IncrementType *pid)
{
	float dErrP,dErrI,dErrD;

	dErrP = pid->errNow-pid->errold1;
	dErrI = pid->errNow;
	dErrD = pid->errNow - 2*pid->errold1 + pid->errold2;

	pid->errold2 = pid->errold1;
	pid->errold1 = pid->errNow;

	pid->dCtrOut = pid->kp * dErrP + pid->ki * dErrI + pid->kd*dErrD;

	if(pid->kp==0&&pid->ki==0&&pid->kd==0)
		pid->ctrOut=0;
	else
		pid->ctrOut += pid->dCtrOut;
	if(pid->MaxOut != 0){
		if(pid->ctrOut > pid->MaxOut) pid->ctrOut = pid->MaxOut;
		if(pid->ctrOut < - pid->MaxOut) pid->ctrOut = -pid->MaxOut;
	}
}

/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
	PID_Typedef *pid,
	uint32_t errlLim,
	uint32_t maxout,
	float kp,
	float ki,
	float kd)
{
  pid->errILim = errlLim;
  pid->MaxOut = maxout;
  pid ->kp=kp;
  pid->ki  =ki;
  pid->kd =kd;
}
void pid_adjust(
	PID_Typedef *pid,
	float kp,
	float ki,
	float kd)
{
	pid->kp  = kp;
	pid->ki   =  ki;
	pid->kd  = kd;
}
//out  0 200
//in     5000 25000

