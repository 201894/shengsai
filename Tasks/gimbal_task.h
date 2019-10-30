/** @file gimbal_task.h
 *  @version 1.0
 *  @date Sep 2019
 *
 *  @brief gimbal control task
 *
 *
 */
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#define pid_fric_adjust       0
#define pid_yaw_adjust      0

#define  fric_kp                                20.0f
#define  fric_ki                                  0.0f
#define  fric_kd                                 0.0f
#define  fric_errILim                          3000
#define  fric_maxOut                         15000

void gimbal_param_init(void);
void yaw_mode_handle(int mode_flag);
#endif