#ifndef PID_H_
#define PID_H_

#include "mpu6050.h"

extern float pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll;
extern float pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
extern float pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw;
extern int   pid_max_roll, pid_max_pitch, pid_max_yaw;

extern float pid_setpoint_roll, pid_setpoint_pitch, pid_setpoint_yaw;
extern float pid_output_roll, pid_output_pitch, pid_output_yaw;
extern float pid_last_error_roll, pid_last_error_pitch, pid_last_error_yaw;
extern float pid_current_error_roll, pid_current_error_pitch, pid_current_error_yaw;
extern float pid_p_output_roll, pid_p_output_pitch, pid_p_output_yaw;
extern float pid_i_output_roll, pid_i_output_pitch, pid_i_output_yaw;
extern float pid_d_output_roll, pid_d_output_pitch, pid_d_output_yaw;

typedef enum {
  PID_SUCCESS = 0,
  PID_ERROR,
} pid_result_t;

pid_result_t pid_calculate();

#endif  /* PID_H_ */