// Changed to English because of uni-code problem //
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
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
#define   SAMPLING_FREQ     (double)( 200.0 )
#define   SAMPLING_TIME     (double)( 1.0 / SAMPLING_FREQ )
#define   HOLD_TIME         (double)( 4.0 )          // voltage hold time per step [sec]
#define   N_HOLD            (int)   ( HOLD_TIME * SAMPLING_FREQ + 100 )

#define   N_BIAS            (int)   ( 200 )
#define   UNIT_PI           (double)( 3.14159265358979 )

#define   K_GIMBAL          (double)( 1000.0 / 0.67 * UNIT_PI / 180.0 )

   /* =====================================================================
      [Voltage Step Sequence]
      - Outside deadzone (0~2V, 3~5V): 0.05V step
      - Deadzone (2~3V):               0.01V step
      CW/CCW alternating from 2.5V center
      ===================================================================== */

      /* Maximum possible steps:
           Outer range: 2.0V / 0.05V = 40 steps per side x2 = 80
           Deadzone:    1.0V / 0.01V = 100 steps
           Total = 180 steps
      */
#define   N_STEPS_MAX       200   // with margin

int BuildVoltageSequence(double voltSeq[N_STEPS_MAX])
{
    // Build sorted absolute voltage list for one side (above 2.5V),
    // then mirror for the other side, alternating CW/CCW.
    //
    // Strategy: generate deltas from 2.5V center in ascending order,
    // alternate +delta (CW) and -delta (CCW).

    // Step 1: collect all unique positive deltas
    double deltas[N_STEPS_MAX];
    int    nDeltas = 0;

    // Outer upper range: 2.5V -> 5.0V  (delta 0.05 to 2.5, step 0.05)
    // But split: delta 0.05~0.45 is outer, delta 0.50 is boundary
    // Deadzone: |Vcmd - 2.5| <= 0.5  =>  delta 0.01 ~ 0.50 (step 0.01)
    // Outer:    delta 0.55 ~ 2.50    (step 0.05)

    // Deadzone deltas: 0.01, 0.02, ..., 0.50
    for (int i = 1; i <= 50; i++)
    {
        deltas[nDeltas++] = i * 0.01;
    }
    // Outer deltas: 0.55, 0.60, ..., 2.50
    for (int i = 1; i <= 39; i++)
    {
        deltas[nDeltas++] = 0.50 + i * 0.05;
    }
    // delta = 2.50 -> Vcmd = 5.0 or 0.0 (boundary)
    deltas[nDeltas++] = 2.50;

    // Remove duplicate 2.50 if any (nDeltas already handles it above;
    // last outer step 0.50 + 39*0.05 = 0.50+1.95 = 2.45, so 2.50 is new)

    // Step 2: interleave CW (+delta) and CCW (-delta)
    int nSteps = 0;
    for (int i = 0; i < nDeltas; i++)
    {
        double vCW = 2.5 + deltas[i];
        double vCCW = 2.5 - deltas[i];

        // Clamp to [0, 5]
        if (vCW > 5.0) vCW = 5.0;
        if (vCCW < 0.0) vCCW = 0.0;

        voltSeq[nSteps++] = vCW;   // CW  (even index)
        voltSeq[nSteps++] = vCCW;  // CCW (odd index)
    }

    return nSteps;
}

/* =====================================================================
   [CalculateGyroBias]
   ===================================================================== */
double CalculateGyroBias(TaskHandle taskAI, int nSamples)
{
    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;
    double  y_bar = 0.0;

    printf("[Gyro Bias Estimation] Collecting %d samples...\n", nSamples);

    for (int k = 1; k <= nSamples; k++)
    {
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
    int32   error;
    double  time_curr = 0.0;
    double  time_init_step = 0.0;
    double  time_elapsed = 0.0;

    TaskHandle taskAI = 0;
    TaskHandle taskAO0 = 0;
    TaskHandle taskAO1 = 0;

    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;

    /* Output directory */
    const char* outputDir = "motor_sweep_data";
    _mkdir(outputDir);
    printf("Output folder: %s\n\n", outputDir);

    /* Build voltage sequence */
    double voltSeq[N_STEPS_MAX];
    int    N_STEPS = BuildVoltageSequence(voltSeq);

    /* Data buffer */
    double bufTime[N_HOLD];
    double bufVcmd[N_HOLD];
    double bufVg[N_HOLD];
    double bufPot[N_HOLD];
    double bufOmega[N_HOLD];

    /* =================================================================
       1. Create tasks
    ================================================================= */
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO0);
    DAQmxCreateTask("", &taskAO1);

    DAQmxCreateAIVoltageChan(taskAI, "Dev3/ai2, Dev3/ai3", "",
        DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO0, "Dev3/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev3/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO0);
    DAQmxStartTask(taskAO1);

    /* =================================================================
       2. Initialize
    ================================================================= */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // switch OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // motor stop

    printf("============================================================\n");
    printf("  Motor Modeling - Voltage Sweep (%d Steps)\n", N_STEPS);
    printf("  Deadzone : 2.0~3.0V  -> 0.01V step, 4sec hold\n");
    printf("  Outer    : 0.0~2.0V, 3.0~5.0V -> 0.05V step, 4sec hold\n");
    printf("  K_gimbal = %.4f [(rad/s)/V]\n", K_GIMBAL);
    printf("============================================================\n\n");

    /* =================================================================
       3. Print voltage sequence
    ================================================================= */
    printf("[Voltage Sequence (Total %d steps)]\n", N_STEPS);
    for (int i = 0; i < N_STEPS; i++)
    {
        const char* zone = (voltSeq[i] >= 2.0 && voltSeq[i] <= 3.0)
            ? "DEAD" : "OUT ";
        printf("  Step %3d: Vcmd = %.2f V  (%s)  [%s]\n",
            i + 1, voltSeq[i],
            (i % 2 == 0) ? "CW " : "CCW", zone);
    }
    printf("\n");

    /* =================================================================
       4. Gyro bias estimation
    ================================================================= */
    printf("[Step 0] Gyro bias estimation - confirm motor stopped, press any key.\n");
    getchar();
    double Vg_offset = CalculateGyroBias(taskAI, N_BIAS);

    /* =================================================================
       5. Start prompt
    ================================================================= */
    printf("[Step 1] Turn on the gimbal switch, then press any key.\n");
    printf("  * Emergency stop: Spacebar\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    /* =================================================================
       6. Voltage step loop
    ================================================================= */
    int emergencyStop = 0;

    for (int step = 0; step < N_STEPS && !emergencyStop; step++)
    {
        double      Vcmd = voltSeq[step];
        const char* dir = (step % 2 == 0) ? "CW" : "CCW";
        const char* zone = (Vcmd >= 2.0 && Vcmd <= 3.0) ? "DEADZONE" : "OUTER";

        printf("-----------------------------------------\n");
        printf("[Step %3d/%d]  Vcmd = %.2f V  (%s)  [%s]\n",
            step + 1, N_STEPS, Vcmd, dir, zone);

        /* Apply voltage */
        DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 3.0, NULL);
        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vcmd, NULL);

        time_init_step = GetWindowTime();
        int count = 0;

        while (1)
        {
            if (GetAsyncKeyState(VK_SPACE) & 0x8000)
            {
                printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
                emergencyStop = 1;
                break;
            }

            time_elapsed = (GetWindowTime() - time_init_step) * 0.001;
            if (time_elapsed >= HOLD_TIME) break;

            error = DAQmxReadAnalogF64(taskAI, 1, 10.0,
                DAQmx_Val_GroupByChannel,
                readArray, 2, &sampsPerChanRead, NULL);
            if (error != 0)
            {
                char errBuff[2048];
                DAQmxGetExtendedErrorInfo(errBuff, 2048);
                printf("DAQ read error: %s\n", errBuff);
            }

            double Vg = readArray[0];
            double Vpot = readArray[1];
            double omega = K_GIMBAL * (Vg - Vg_offset);

            if (count < N_HOLD)
            {
                bufTime[count] = time_elapsed;
                bufVcmd[count] = Vcmd;
                bufVg[count] = Vg;
                bufPot[count] = Vpot;
                bufOmega[count] = omega;
                count++;
            }

            while (1)
            {
                time_curr = GetWindowTime();
                if (time_curr - time_init_step - (count - 1) * SAMPLING_TIME * 1000.0
                    >= SAMPLING_TIME * 1000.0) break;
            }
        }

        /* Save to file */
        if (count > 0)
        {
            char filename[256];
            sprintf(filename, "%s/step_%03d_V%.2f_%s.out",
                outputDir, step + 1, Vcmd, dir);

            FILE* pFile = fopen(filename, "w+t");
            if (pFile)
            {
                fprintf(pFile, "%% Motor Sweep Step %d/%d\n", step + 1, N_STEPS);
                fprintf(pFile, "%% Vcmd       = %.4f [V]\n", Vcmd);
                fprintf(pFile, "%% Direction  = %s\n", dir);
                fprintf(pFile, "%% Zone       = %s\n", zone);
                fprintf(pFile, "%% Vg_offset  = %.6f [V]\n", Vg_offset);
                fprintf(pFile, "%% K_gimbal   = %.6f [(rad/s)/V]\n\n", K_GIMBAL);
                fprintf(pFile, "Time[s]              Vcmd[V]              "
                    "Vg_raw[V]            Pot[V]               Omega[rad/s]\n");

                for (int i = 0; i < count; i++)
                {
                    fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f\n",
                        bufTime[i], bufVcmd[i], bufVg[i],
                        bufPot[i], bufOmega[i]);
                }
                fclose(pFile);
                printf("  -> Saved: %s  (%d samples)\n", filename, count);
            }
            else
            {
                printf("  !! Failed to open file: %s\n", filename);
            }
        }

        /* Neutral pause between steps */
        if (!emergencyStop && step < N_STEPS - 1)
        {
            DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);
            Sleep(1000);
        }
    }

    /* =================================================================
       7. Shutdown
    ================================================================= */
    printf("\n============================================================\n");
    printf("  Experiment finished - stopping motor.\n");
    printf("============================================================\n");

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    DAQmxStopTask(taskAI);   DAQmxClearTask(taskAI);
    DAQmxStopTask(taskAO0);  DAQmxClearTask(taskAO0);
    DAQmxStopTask(taskAO1);  DAQmxClearTask(taskAO1);

    printf("\n[Done] All data saved to '%s' folder.\n", outputDir);
    printf("Vg_offset = %.6f V,  K_gimbal = %.4f (rad/s)/V\n",
        Vg_offset, K_GIMBAL);
}
