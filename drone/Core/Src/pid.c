#include "pid.h"

float pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll;
float pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
float pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw;
int   pid_max_roll, pid_max_pitch, pid_max_yaw;

float pid_setpoint_roll, pid_setpoint_pitch, pid_setpoint_yaw;
float pid_output_roll, pid_output_pitch, pid_output_yaw;
float pid_last_error_roll, pid_last_error_pitch, pid_last_error_yaw;
float pid_current_error_roll, pid_current_error_pitch, pid_current_error_yaw;
float pid_p_output_roll, pid_p_output_pitch, pid_p_output_yaw;
float pid_i_output_roll, pid_i_output_pitch, pid_i_output_yaw;
float pid_d_output_roll, pid_d_output_pitch, pid_d_output_yaw;


pid_result_t pid_calculate() {
  /* ROLL */
  pid_current_error_roll = gyro_input_roll - pid_setpoint_roll;
  pid_p_output_roll = pid_current_error_roll * pid_p_gain_roll;
  pid_i_output_roll += pid_current_error_roll * pid_i_gain_roll;
  if (pid_i_output_roll > pid_max_roll) {
    pid_i_output_roll = pid_max_roll;
  } else if (pid_i_output_roll < pid_max_roll * (-1)) {
    pid_i_output_roll = pid_max_roll * (-1);
  }
  pid_d_output_roll = (pid_current_error_roll - pid_last_error_roll) * pid_d_gain_roll;

  pid_output_roll = pid_p_output_roll + pid_i_output_roll + pid_d_output_roll;
  if (pid_output_roll > pid_max_roll) {
    pid_output_roll = pid_max_roll;
  } else if (pid_output_roll < pid_max_roll * (-1)) {
    pid_output_roll = pid_max_roll * (-1);
  }

  pid_last_error_roll = pid_current_error_roll;

  /* PITCH */
  pid_current_error_pitch = gyro_input_pitch - pid_setpoint_pitch;
  pid_p_output_pitch = pid_current_error_pitch * pid_p_gain_pitch;
  pid_i_output_pitch += pid_current_error_pitch * pid_i_gain_pitch;
  if (pid_i_output_pitch > pid_max_pitch) {
    pid_i_output_pitch = pid_max_pitch;
  } else if (pid_i_output_pitch < pid_max_pitch * (-1)) {
    pid_i_output_pitch = pid_max_pitch * (-1);
  }
  pid_d_output_pitch = (pid_current_error_pitch - pid_last_error_pitch) * pid_d_gain_pitch;

  pid_output_pitch = pid_p_output_pitch + pid_i_output_pitch + pid_d_output_pitch;
  if (pid_output_pitch > pid_max_pitch) {
    pid_output_pitch = pid_max_pitch;
  } else if (pid_output_pitch < pid_max_pitch * (-1)) {
    pid_output_pitch = pid_max_pitch * (-1);
  }

  pid_last_error_pitch = pid_current_error_pitch;

  /* YAW */
  pid_current_error_yaw = gyro_input_yaw - pid_setpoint_yaw;
  pid_p_output_yaw = pid_current_error_yaw * pid_p_gain_yaw;
  pid_i_output_yaw += pid_current_error_yaw * pid_i_gain_yaw;
  if (pid_i_output_yaw > pid_max_yaw) {
    pid_i_output_yaw = pid_max_yaw;
  } else if (pid_i_output_yaw < pid_max_yaw * (-1)) {
    pid_i_output_yaw = pid_max_yaw * (-1);
  }
  pid_d_output_yaw = (pid_current_error_yaw - pid_last_error_yaw) * pid_d_gain_yaw;

  pid_output_yaw = pid_p_output_yaw + pid_i_output_yaw + pid_d_output_yaw;
  if (pid_output_yaw > pid_max_yaw) {
    pid_output_yaw = pid_max_yaw;
  } else if (pid_output_yaw < pid_max_yaw * (-1)) {
    pid_output_yaw = pid_max_yaw * (-1);
  }

  pid_last_error_yaw = pid_current_error_yaw;

  return PID_SUCCESS;
}
