/*
 * MIT License
 *
 * Copyright (c) 2022 Teledatics, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/**
 * @file teledatics_gui_metrics.c
 * @author James Ewing
 * @date 24 Mar 2022
 * @brief Teledatics HTML GUI for Halo TD-XPAH
 */

#include "teledatics_gui.h"
#include <stdio.h>
#include <stdlib.h>

#define STATS_TASK_PRIO 3
#define STATS_TICKS pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET                                                      \
  5 // Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

#define STATS_UART NRC_UART_CH0
#define STATS_UART_BASE HSUART0_BASE_ADDR
#define STATS_BAUD_RATE 115200

static NRC_UART_CONFIG sio_uart_stats;

void
stats_uart_print(char* fmt, ...)
{
  static char buf[128];
  char* p = NULL;
  va_list ap;

  if (NULL == fmt) {
    return;
  }

  memset(buf, 0, sizeof(buf));

  va_start(ap, fmt);
  if (vsnprintf(buf, sizeof(buf), fmt, ap) < 0) {
    nrc_usr_print("[%s] vsnprintf failed\n", __func__);
    return;
  }
  va_end(ap);

  p = buf;
  do {
    if (nrc_uart_put(STATS_UART, *p) != NRC_SUCCESS) {
      nrc_usr_print("[%s] vsnprintf failed\n", __func__);
      break;
    }
  } while (*p++);
}

void
setup_stats_uart(void)
{
  if (nrc_uart_set_channel(STATS_UART) != NRC_SUCCESS) {
    nrc_usr_print("[%s] nrc_uart_set_channel failed\n", __func__);
    return;
  }

  sio_uart_stats.ch = STATS_UART;
  sio_uart_stats.db = NRC_UART_DB8;                   /**< Data bit */
  sio_uart_stats.br = STATS_BAUD_RATE;                /**< Baud rate */
  sio_uart_stats.stop_bit = NRC_UART_SB1;             /**< Stop bit */
  sio_uart_stats.parity_bit = NRC_UART_PB_NONE;       /**< Parity bit */
  sio_uart_stats.hw_flow_ctrl = NRC_UART_HFC_DISABLE; /**< HW flow control */
  sio_uart_stats.fifo = NRC_UART_FIFO_ENABLE;

  if (nrc_uart_set_config(&sio_uart_stats) != NRC_SUCCESS) {
    nrc_usr_print("[%s] nrc_uart_set_config failed\n", __func__);
    return;
  }
}
/**
 * @brief   Function to print the CPU usage of tasks over a given duration.
 *
 * This function will measure and print the CPU usage of tasks over a specified
 * number of ticks (i.e. real time stats). This is implemented by simply calling
 * uxTaskGetSystemState() twice separated by a delay, then calculating the
 * differences of task run times before and after the delay.
 *
 * @param   xTicksToWait    Period of stats measurement
 *
 * @return -1 on error, 0 on success
 */
static int
print_real_time_stats(TickType_t xTicksToWait)
{
  TaskStatus_t *start_array = NULL, *end_array = NULL;
  UBaseType_t start_array_size, end_array_size;
  uint32_t start_run_time, end_run_time;
  static char buf[128];
  int ret;

  // move to home
  stats_uart_print("%c%c%c", 0x1B, 0x5B, 0x48);

  // Allocate array to store current task states
  start_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
  start_array = malloc(sizeof(TaskStatus_t) * start_array_size);
  if (start_array == NULL) {
    ret = -1;
    goto exit;
  }
  // Get current task states
  start_array_size =
    uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
  if (start_array_size == 0) {
    ret = -1;
    goto exit;
  }

  vTaskDelay(xTicksToWait);

  // Allocate array to store tasks states post delay
  end_array_size = uxTaskGetNumberOfTasks() + ARRAY_SIZE_OFFSET;
  end_array = malloc(sizeof(TaskStatus_t) * end_array_size);
  if (end_array == NULL) {
    ret = -1;
    goto exit;
  }
  // Get post delay task states
  end_array_size =
    uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
  if (end_array_size == 0) {
    ret = -1;
    goto exit;
  }

  // Calculate total_elapsed_time in units of run time stats clock period.
  uint32_t total_elapsed_time = (end_run_time - start_run_time);
  if (total_elapsed_time == 0) {
    ret = -1;
    goto exit;
  }

  snprintf(buf,
           sizeof(buf),
           "| %22s | %10s | %16s |\n",
           "Task",
           "Run Time",
           "Percentage");
  stats_uart_print("%57s", buf);
  // Match each task in start_array to those in the end_array
  for (int i = 0; i < start_array_size; i++) {
    int k = -1;
    for (int j = 0; j < end_array_size; j++) {
      if (start_array[i].xHandle == end_array[j].xHandle) {
        k = j;
        // Mark that task have been matched by overwriting their handles
        start_array[i].xHandle = NULL;
        end_array[j].xHandle = NULL;
        break;
      }
    }
    // Check if matching task found
    if (k >= 0) {
      uint32_t task_elapsed_time =
        end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
      uint32_t percentage_time =
        (task_elapsed_time * 100UL) / total_elapsed_time;
      snprintf(buf,
               sizeof(buf),
               "| %22s | %10ld | %15ld%% |\n",
               start_array[i].pcTaskName,
               task_elapsed_time,
               percentage_time);
      stats_uart_print("%57s", buf);
    }
  }

  // Print unmatched tasks
  for (int i = 0; i < start_array_size; i++) {
    if (start_array[i].xHandle != NULL) {
      snprintf(
        buf, sizeof(buf), "| %22s | Deleted \n", end_array[i].pcTaskName);
      stats_uart_print(buf, "%57s |");
    }
  }
  for (int i = 0; i < end_array_size; i++) {
    if (end_array[i].xHandle != NULL) {
      snprintf(
        buf, sizeof(buf), "| %22s | Created \n", end_array[i].pcTaskName);
      stats_uart_print(buf, "%57s |");
    }
  }
  ret = 0;

exit: // Common return path
  free(start_array);
  free(end_array);
  return ret;
}

static void
stats_task(void* arg)
{
  while (1) {
    print_real_time_stats(STATS_TICKS);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void
runStats(void)
{
  setup_stats_uart();

  // clear screen
  stats_uart_print("%c%c%c%c", 0x1B, 0x5B, 0x32, 0x4A);

  xTaskCreate(stats_task, "stats", 4096, NULL, STATS_TASK_PRIO, NULL);
}
