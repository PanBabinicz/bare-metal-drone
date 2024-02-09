#include "fst6.h"

int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
int32_t channel_1_start, channel_1, channel_2_start, channel_2, channel_3_start, channel_3, channel_4_start, channel_4;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if (0b1 & (GPIOA->IDR >> 6)) {                                  /**< When input is high?. */
      channel_1_start = __HAL_TIM_GET_COUNTER(htim);
    } else if (!(0b1 & (GPIOA->IDR >> 6))) {                        /**< When input is low?. */
      channel_1 = __HAL_TIM_GET_COUNTER(htim) - channel_1_start;
      if (channel_1 < 0) {
        channel_1 += 0xFFFF;
      }
    }
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    if (0b1 & (GPIOA->IDR >> 7)) {
      channel_2_start = __HAL_TIM_GET_COUNTER(htim);
    } else if (!(0b1 & (GPIOA->IDR >> 7))) {
      channel_2 = __HAL_TIM_GET_COUNTER(htim) - channel_2_start;
      if (channel_2 < 0) {
        channel_2 += 0xFFFF;
      }
    }
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    if (0b1 & (GPIOB->IDR >> 0)) {
      channel_3_start = __HAL_TIM_GET_COUNTER(htim);
    } else if (!(0b1 & (GPIOB->IDR >> 0))) {
      channel_3 = __HAL_TIM_GET_COUNTER(htim) - channel_3_start;
      if (channel_3 < 0) {
        channel_3 += 0xFFFF;
      }
    }
  } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
    if (0b1 & (GPIOB->IDR >> 1)) {
      channel_4_start = __HAL_TIM_GET_COUNTER(htim);
    } else if (!(0b1 & (GPIOB->IDR >> 1))) {
      channel_4 = __HAL_TIM_GET_COUNTER(htim) - channel_4_start;
      if (channel_4 < 0) {
        channel_4 += 0xFFFF;
      }
    }
  }
}

fst6_result_t fst6_capture_signal_tim3(fst6_callback_t fst6_callback) {
    fst6_callback(&htim3);
    return FST6_SUCCESS;
}

/*
fst6_result_t fst6_print_signals_tim2_channel_1() {
    uint8_t message[100];

    strcpy((char*)message, "Roll: ");
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    if (channel_1 - 1480 < 0) {
        strcpy((char*)message, "<<<");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else if (channel_1 - 1520 > 0) {
        strcpy((char*)message, ">>>");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else {
        strcpy((char*)message, "-+-");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    }
    sprintf((char*)message, "%ld   ", channel_1);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    return FST6_SUCCESS;
}

fst6_result_t fst6_print_signals_tim2_channel_2() {
    uint8_t message[100];

    strcpy((char*)message, "Pitch: ");
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    if (channel_2 - 1480 < 0) {
        strcpy((char*)message, "↓↓↓");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else if (channel_2 - 1520 > 0) {
        strcpy((char*)message, "↑↑↑");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else {
        strcpy((char*)message, "-+-");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    }
    sprintf((char*)message, "%ld   ", channel_2);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    return FST6_SUCCESS;
}

fst6_result_t fst6_print_signals_tim2_channel_3() {
    uint8_t message[100];

    strcpy((char*)message, "Throttle: ");
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    if (channel_3 - 1480 < 0) {
        strcpy((char*)message, "---");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else if (channel_3 - 1520 > 0) {
        strcpy((char*)message, "+++");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else {
        strcpy((char*)message, "-+-");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    }
    sprintf((char*)message, "%ld   ", channel_3);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    return FST6_SUCCESS;
}

fst6_result_t fst6_print_signals_tim2_channel_4() {
    uint8_t message[100];

    strcpy((char*)message, "Yaw: ");
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    if (channel_4 - 1480 < 0) {
        strcpy((char*)message, "<<<");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else if (channel_4 - 1520 > 0) {
        strcpy((char*)message, ">>>");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    } else {
        strcpy((char*)message, "-+-");
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);
    }
    sprintf((char*)message, "%ld\r\n", channel_4);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen((char*)message), 300);

    return FST6_SUCCESS;
}

fst6_result_t fst6_print_signals_tim2_channel_all() {
    fst6_print_signals_tim2_channel_1();
    fst6_print_signals_tim2_channel_2();
    fst6_print_signals_tim2_channel_3();
    fst6_print_signals_tim2_channel_4();

    return FST6_SUCCESS;
}
*/