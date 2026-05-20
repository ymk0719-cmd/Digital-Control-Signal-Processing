#ifndef _MACRO_H
#define _MACRO_H

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include <string.h>

#define DAQ_DEV         "Dev3"
#define NEUTRAL         (float64)(2.5)
#define ON              (float64)(5.0)
#define OFF             (float64)(0.0)
#define SAMPLING_FREQ   (double)(200.0)
#define SAMPLING_TIME   (double)(1.0 / SAMPLING_FREQ)
#define UNIT_PI         (double)(3.14159265358979)
#define K_GIMBAL        (double)(1000.0 / 0.67 * UNIT_PI / 180.0)
#define N_BIAS          (int)(200)
#define BUF_SIZE        (1000)

#define READ_DATA(arr)  DAQmxReadAnalogF64(g_taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, (arr), 2, &sampsPerChanRead, NULL)

#define EXIT            (0)
#define VOLTAGE_SWEEP   (1)
#define TRI_VALIDATION  (2)
#define SINE_VALIDATION (3)
#define FREQ_SWEEP      (4)

#define K_LIN           (double)( 8.0353)      /* <---- MODIFY!! */
#define CW_C2           (double)(-2.6952)      /* <---- MODIFY!! */
#define CW_C1           (double)(31.0266)      /* <---- MODIFY!! */
#define CW_C0           (double)(-64.6768)     /* <---- MODIFY!! */
#define CCW_C2          (double)(2.3433)      /* <---- MODIFY!! */
#define CCW_C1          (double)( 4.9178)      /* <---- MODIFY!! */
#define CCW_C0          (double)(-23.1329)     /* <---- MODIFY!! */
#define DEAD_THRESH     (double)(0.5)

#define HOLD_TIME       (double)(4.0)
#define N_HOLD          (int)(HOLD_TIME * SAMPLING_FREQ + 100)
#define N_STEPS_MAX     200

#define TRI_AMP         (double)(1)
#define TRI_PERIOD      (double)(40.0)
#define TRI_CYCLES      (int)(5)
#define TRI_T_TOTAL     (double)(TRI_PERIOD * TRI_CYCLES)
#define TRI_N_MAX       (int)(TRI_T_TOTAL * SAMPLING_FREQ + 200)

#define SINE_AMP        (double)(1)
#define SINE_FREQ       (double)(0.025)
#define SINE_PERIOD     (double)(1.0 / SINE_FREQ)
#define SINE_CYCLES     (int)(5)
#define SINE_T_TOTAL    (double)(SINE_PERIOD * SINE_CYCLES)
#define SINE_N_MAX      (int)(SINE_T_TOTAL * SAMPLING_FREQ + 200)
#define SINE_CMD(t)     (SINE_AMP * sin(2.0 * UNIT_PI * SINE_FREQ * (t)))

#define MODE_SINE       (0)
#define MODE_TRI        (1)

#define BODE_SINE_AMP   (double)(1)
#define N_FREQS         (12)
#define N_CYCLES_LOW    (int)(5)
#define N_CYCLES_MID    (int)(10)
#define N_CYCLES_HIGH   (int)(20)
#define FREQ_THR_LOW    (double)(0.5)
#define FREQ_THR_MID    (double)(2.0)
#define BODE_N_MAX      (int)(12000)

#endif
