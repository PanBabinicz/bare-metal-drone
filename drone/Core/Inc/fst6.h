#ifndef FST6_H_
#define FST6_H_

#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "stm32f411xe.h"

extern TIM_HandleTypeDef htim3;
extern int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
extern int32_t channel_1_start, channel_1, channel_2_start, channel_2, channel_3_start, channel_3, channel_4_start, channel_4;

typedef enum {
    FST6_SUCCESS = 0,
    FST6_ERROR,
} fst6_result_t;

typedef void (*fst6_callback_t)(TIM_HandleTypeDef *);

fst6_result_t fst6_capture_signal_tim3(fst6_callback_t fst6_callback);
fst6_result_t fst6_print_signals_tim3_channel_1();
fst6_result_t fst6_print_signals_tim3_channel_2();
fst6_result_t fst6_print_signals_tim3_channel_3();
fst6_result_t fst6_print_signals_tim3_channel_4();
fst6_result_t fst6_print_signals_tim3_channel_all();

#endif  /* FST6_H_ */
