// =====================================================================
//  motor_triangle_verify.c
//  Motor Linearization Verification - Triangle Wave Input
//  Inverse mapping computed inline 
// 
//  Before compiling:
//    1. Run MATLAB script to get p_cw, p_ccw, K
//    2. Fill in the TODO section below with those values
//
//  Signal design:
//    Vcmd_ref : triangle wave, amplitude = 1.5 V, period = 10 sec
//    Duration : 5 cycles = 50 sec
//    Fs       : 200 Hz
// =====================================================================
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include "NIDAQmx.h"

/* =====================================================================
   [Time Function] Returns current time in ms
   ===================================================================== */
double GetWindowTime(void)
{
    LARGE_INTEGER liCounter, liFrequency;
    QueryPerformanceCounter(&liCounter);
    QueryPerformanceFrequency(&liFrequency);
    return (liCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}

/* =====================================================================
   [Macro Definitions]
   ===================================================================== */
#define SAMPLING_FREQ   (double)(200.0)
#define SAMPLING_TIME   (double)(1.0 / SAMPLING_FREQ)

#define TRI_AMPLITUDE   (double)(2.5)
#define TRI_PERIOD      (double)(40)
#define TRI_CYCLES      (int)  (5)
#define T_TOTAL         (double)(TRI_PERIOD * TRI_CYCLES)   // 50 sec

#define N_SAMPLES_MAX   (int)(T_TOTAL * SAMPLING_FREQ + 200)

#define N_BIAS          (int)(200)
#define UNIT_PI         (double)(3.14159265358979)
#define K_GIMBAL        (double)(1000.0 / 0.67 * UNIT_PI / 180.0)

   /* =====================================================================
      TODO: Fill in values printed by MATLAB before compiling
      =====================================================================
      Run MATLAB and copy the printed values here:

        fprintf('[CW  fitting] %.4f %.4f %.4f\n', p_cw(1),  p_cw(2),  p_cw(3));
        fprintf('[CCW fitting] %.4f %.4f %.4f\n', p_ccw(1), p_ccw(2), p_ccw(3));
        fprintf('K = %.4f\n', K);

      CW  branch valid Vc range : [2.5, 5.0] V
      CCW branch valid Vc range : [0.0, 2.5] V
      Deadzone: |omega_target| <= DEAD_THRESH -> Vc = 2.5 V
      ===================================================================== */

      /* Linearization gain K [(rad/s)/V] */
#define K_LIN       (double)(8.3331)       // TODO: replace with MATLAB K

/* CW polynomial:  omega = P_CW_A*Vc^2 + P_CW_B*Vc + P_CW_C */
#define P_CW_A      (double)(-3.8629)       // TODO: replace with p_cw(1)
#define P_CW_B      (double)(39.9215)       // TODO: replace with p_cw(2)
#define P_CW_C      (double)(-80.2505)       // TODO: replace with p_cw(3)

/* CCW polynomial: omega = P_CCW_A*Vc^2 + P_CCW_B*Vc + P_CCW_C */
#define P_CCW_A     (double)(3.5534 )       // TODO: replace with p_ccw(1)
#define P_CCW_B     (double)(1.9357)       // TODO: replace with p_ccw(2)
#define P_CCW_C     (double)(-22.6890)       // TODO: replace with p_ccw(3)

/* Deadzone threshold [rad/s] */
#define DEAD_THRESH (double)(0.5)

/* =====================================================================
   [InverseMap]
   Computes Vc from Vcmd_ref via inline inverse polynomial mapping.
   Mirrors MATLAB Section 6 logic exactly.

     1. omega_target = K_LIN * vcmd_ref
     2. CW  (omega >  DEAD_THRESH): solve CW  quadratic, pick root in [2.5, 5.0]
        CCW (omega < -DEAD_THRESH): solve CCW quadratic, pick root in [0.0, 2.5]
        Deadzone                  : Vc = 2.5 V
     3. Clamp Vc to [0, 5]
   ===================================================================== */
double InverseMap(double vcmd_ref)
{
    double omega_target = K_LIN * vcmd_ref;
    double Vc = 2.5;

    if (omega_target > DEAD_THRESH)
    {
        /* CW branch */
        double a = P_CW_A;
        double b = P_CW_B;
        double c = P_CW_C - omega_target;
        double disc = b * b - 4.0 * a * c;

        if (disc >= 0.0)
        {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int    r1ok = (r1 >= 2.5) && (r1 <= 5.0);
            int    r2ok = (r2 >= 2.5) && (r2 <= 5.0);

            if (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;  // pick smaller Vc
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }
    else if (omega_target < -DEAD_THRESH)
    {
        /* CCW branch */
        double a = P_CCW_A;
        double b = P_CCW_B;
        double c = P_CCW_C - omega_target;
        double disc = b * b - 4.0 * a * c;

        if (disc >= 0.0)
        {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int    r1ok = (r1 >= 0.0) && (r1 <= 2.5);
            int    r2ok = (r2 >= 0.0) && (r2 <= 2.5);

            if (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;  // pick smaller Vc
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }
    /* else: deadzone, Vc = 2.5 */

    if (Vc < 0.0) Vc = 0.0;
    if (Vc > 5.0) Vc = 5.0;
    return Vc;
}

/* =====================================================================
   [TriangleWave]
   Starts at 0 V, rises to +A at T/4, falls to -A at 3T/4, back to 0 at T
   ===================================================================== */
double TriangleWave(double t, double amplitude, double period)
{
    double phase = fmod(t, period) / period;   // [0, 1)

    if (phase < 0.25)
        return  amplitude * (4.0 * phase);
    else if (phase < 0.75)
        return  amplitude * (2.0 - 4.0 * phase);
    else
        return  amplitude * (4.0 * phase - 4.0);
}

/* =====================================================================
   [CalculateGyroBias]
   ===================================================================== */
double CalculateGyroBias(TaskHandle taskAI, int nSamples)
{
    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;
    double  y_bar = 0.0;

    printf("[Gyro Bias] Collecting %d samples...\n", nSamples);
    for (int k = 1; k <= nSamples; k++) {
        DAQmxReadAnalogF64(taskAI, 1, 10.0, DAQmx_Val_GroupByChannel,
            readArray, 2, &sampsPerChanRead, NULL);
        double y_k = readArray[0];
        y_bar = (1.0 - 1.0 / k) * y_bar + (1.0 / k) * y_k;
        Sleep(5);
    }
    printf("[Gyro Bias Done] Vg_offset = %.6f V\n\n", y_bar);
    return y_bar;
}

/* =====================================================================
   [main]
   ===================================================================== */
void main(void)
{
    int32  error;
    double time_curr = 0.0;
    double time_start = 0.0;
    double time_elapsed = 0.0;

    TaskHandle taskAI = 0;
    TaskHandle taskAO0 = 0;
    TaskHandle taskAO1 = 0;

    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;

    /* Output directory */
    const char* outputDir = "triangle_verify_data";
    _mkdir(outputDir);

    /* ---------------------------------------------------------------
       Allocate data buffers on heap
    --------------------------------------------------------------- */
    double* bufTime = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufVcmd = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufVc = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufVg = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufPot = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufOmega = (double*)malloc(N_SAMPLES_MAX * sizeof(double));
    double* bufOmegaTarget = (double*)malloc(N_SAMPLES_MAX * sizeof(double));

    if (!bufTime || !bufVcmd || !bufVc || !bufVg ||
        !bufPot || !bufOmega || !bufOmegaTarget)
    {
        printf("[ABORT] Memory allocation failed.\n");
        return;
    }

    /* ---------------------------------------------------------------
       Create DAQ tasks
    --------------------------------------------------------------- */
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO0);
    DAQmxCreateTask("", &taskAO1);

    DAQmxCreateAIVoltageChan(taskAI, "Dev3/ai2, Dev3/ai3", "",
        DAQmx_Val_RSE, -10.0, 10.0,
        DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO0, "Dev3/ao0", "",
        0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev3/ao1", "",
        0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO0);
    DAQmxStartTask(taskAO1);

    /* Initialize: switch OFF, motor neutral */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    printf("============================================================\n");
    printf("  Motor Linearization Verification - Triangle Wave\n");
    printf("  Amplitude  : +/-%.1f V\n", TRI_AMPLITUDE);
    printf("  Period     : %.1f sec\n", TRI_PERIOD);
    printf("  Cycles     : %d\n", TRI_CYCLES);
    printf("  Duration   : %.1f sec\n", T_TOTAL);
    printf("  Fs         : %.0f Hz\n", SAMPLING_FREQ);
    printf("  K_lin      : %.4f (rad/s)/V\n", K_LIN);
    printf("  K_gimbal   : %.4f (rad/s)/V\n", K_GIMBAL);
    printf("  Dead thresh: %.2f rad/s\n", DEAD_THRESH);
    printf("  p_cw       : [%.4f, %.4f, %.4f]\n", P_CW_A, P_CW_B, P_CW_C);
    printf("  p_ccw      : [%.4f, %.4f, %.4f]\n", P_CCW_A, P_CCW_B, P_CCW_C);
    printf("  Output     : %s/\n", outputDir);
    printf("============================================================\n\n");

    /* ---------------------------------------------------------------
       Gyro bias estimation
    --------------------------------------------------------------- */
    printf("[Step 0] Confirm motor is stopped, then press Enter.\n");
    getchar();
    double Vg_offset = CalculateGyroBias(taskAI, N_BIAS);

    /* ---------------------------------------------------------------
       Start prompt
    --------------------------------------------------------------- */
    printf("[Step 1] Turn on the gimbal switch, then press Enter.\n");
    printf("  * Emergency stop: Spacebar\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    /* ---------------------------------------------------------------
       Triangle wave measurement loop
    --------------------------------------------------------------- */
    printf("[START] Running triangle wave experiment (%.0f sec)...\n\n", T_TOTAL);

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 3.0, NULL);  // switch ON

    time_start = GetWindowTime();
    int    count = 0;
    int    emergStop = 0;
    double last_print_t = -1.0;

    while (1)
    {
        /* Emergency stop */
        if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
            printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
            emergStop = 1;
            break;
        }

        time_elapsed = (GetWindowTime() - time_start) * 0.001;
        if (time_elapsed >= T_TOTAL) break;

        /* Progress report every 10 sec */
        if (time_elapsed - last_print_t >= 10.0) {
            int cycle_now = (int)(time_elapsed / TRI_PERIOD) + 1;
            printf("  t = %5.1f sec  (cycle %d/%d)\n",
                time_elapsed, cycle_now, TRI_CYCLES);
            last_print_t = time_elapsed;
        }

        /* Triangle wave command */
        double vcmd_ref = TriangleWave(time_elapsed, TRI_AMPLITUDE, TRI_PERIOD);

        /* Inline inverse mapping: Vcmd_ref -> Vc */
        double Vc = InverseMap(vcmd_ref);

        /* Target omega */
        double omega_target = K_LIN * vcmd_ref;

        /* Apply Vc */
        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vc, NULL);

        /* DAQ read */
        error = DAQmxReadAnalogF64(taskAI, 1, 10.0,
            DAQmx_Val_GroupByChannel,
            readArray, 2,
            &sampsPerChanRead, NULL);
        if (error != 0) {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("DAQ read error: %s\n", errBuff);
        }

        double Vg = readArray[0];
        double Vpot = readArray[1];
        double omega = K_GIMBAL * (Vg - Vg_offset);

        /* Store */
        if (count < N_SAMPLES_MAX) {
            bufTime[count] = time_elapsed;
            bufVcmd[count] = vcmd_ref;
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufPot[count] = Vpot;
            bufOmega[count] = omega;
            bufOmegaTarget[count] = omega_target;
            count++;
        }

        /* Wait for next sampling interval */
        while (1) {
            time_curr = GetWindowTime();
            if ((time_curr - time_start) * 0.001
                - (count - 1) * SAMPLING_TIME
                >= SAMPLING_TIME) break;
        }
    }

    /* Motor to neutral */
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    printf("\n[DONE] Collected %d samples  (%.2f sec)\n\n",
        count, (count > 0 ? bufTime[count - 1] : 0.0));

    /* ---------------------------------------------------------------
       Save result file
    --------------------------------------------------------------- */
    char filename[256];
    sprintf(filename, "%s/triangle_verify_A%.1f_T%.0f_%dcyc.out",
        outputDir, TRI_AMPLITUDE, TRI_PERIOD, TRI_CYCLES);

    FILE* pFile = fopen(filename, "w+t");
    if (pFile)
    {
        fprintf(pFile,
            "%% Motor Triangle Wave Linearization Verification\n"
            "%% Amplitude    = %.4f [V]\n"
            "%% Period       = %.4f [sec]\n"
            "%% Cycles       = %d\n"
            "%% Duration     = %.4f [sec]\n"
            "%% K_lin        = %.6f [(rad/s)/V]\n"
            "%% K_gimbal     = %.6f [(rad/s)/V]\n"
            "%% Vg_offset    = %.6f [V]\n"
            "%% p_cw         = [%.6f, %.6f, %.6f]\n"
            "%% p_ccw        = [%.6f, %.6f, %.6f]\n\n",
            TRI_AMPLITUDE, TRI_PERIOD, TRI_CYCLES, T_TOTAL,
            K_LIN, K_GIMBAL, Vg_offset,
            P_CW_A, P_CW_B, P_CW_C,
            P_CCW_A, P_CCW_B, P_CCW_C);

        fprintf(pFile,
            "Time[s]              Vcmd_ref[V]          "
            "Vc[V]                Vg_raw[V]            "
            "Pot[V]               Omega[rad/s]         "
            "Omega_target[rad/s]\n");

        for (int i = 0; i < count; i++) {
            fprintf(pFile,
                "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
                bufTime[i], bufVcmd[i], bufVc[i],
                bufVg[i], bufPot[i], bufOmega[i],
                bufOmegaTarget[i]);
        }
        fclose(pFile);
        printf("[Saved] %s  (%d rows)\n", filename, count);
    }
    else
    {
        printf("[ERROR] Failed to open output file: %s\n", filename);
    }

    /* ---------------------------------------------------------------
       RMS / peak error statistics
    --------------------------------------------------------------- */
    if (count > 0 && !emergStop)
    {
        double rms = 0.0;
        double peak_err = 0.0;
        for (int i = 0; i < count; i++) {
            double e = bufOmega[i] - bufOmegaTarget[i];
            rms += e * e;
            if (fabs(e) > fabs(peak_err)) peak_err = e;
        }
        rms = sqrt(rms / count);

        printf("\n------------------------------------------------------------\n");
        printf("  Verification Statistics (%d samples)\n", count);
        printf("  RMS error  : %.4f rad/s\n", rms);
        printf("  Peak error : %+.4f rad/s\n", peak_err);
        printf("------------------------------------------------------------\n");
    }

    /* ---------------------------------------------------------------
       Shutdown
    --------------------------------------------------------------- */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    DAQmxStopTask(taskAI);  DAQmxClearTask(taskAI);
    DAQmxStopTask(taskAO0); DAQmxClearTask(taskAO0);
    DAQmxStopTask(taskAO1); DAQmxClearTask(taskAO1);

    free(bufTime);  free(bufVcmd);  free(bufVc);
    free(bufVg);    free(bufPot);   free(bufOmega);
    free(bufOmegaTarget);

    printf("\n[Done] Output folder: '%s/'\n", outputDir);
    printf("  Vg_offset : %.6f V\n", Vg_offset);
    printf("  K_lin     : %.4f (rad/s)/V\n", K_LIN);
    printf("  K_gimbal  : %.4f (rad/s)/V\n", K_GIMBAL);
}
