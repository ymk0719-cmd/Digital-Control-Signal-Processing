//Changed to English because of uni-code problem //
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include <direct.h>   // _mkdir
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
} // [ms]

/* =====================================================================
   [Macro Definitions]
   ===================================================================== */
#define   SAMPLING_FREQ     (double)( 200.0 )
#define   SAMPLING_TIME     (double)( 1.0 / SAMPLING_FREQ )
#define   HOLD_TIME         (double)( 10.0 )         // voltage hold time per step [sec]
#define   N_HOLD            (int)   ( HOLD_TIME * SAMPLING_FREQ + 100 )  // buffer with margin

#define   N_BIAS            (int)   ( 200 )           // number of samples for bias estimation (1 sec)
#define   UNIT_PI           (double)( 3.14159265358979 )

   // -----------------------------------------------------------------------
   // [Gyro Scale Factor]
   //   k_g = 0.67 [mV/(deg/s)]
   //   omega_h [rad/s] = (V_g - V_g_offset) [V] x 1000 [mV/V] / 0.67 [mV/(deg/s)] x (pi/180)
   // -----------------------------------------------------------------------
#define   K_GIMBAL          (double)( 1000.0 / 0.67 * UNIT_PI / 180.0 )  // ~26.05 [(rad/s)/V]

/* =====================================================================
   [Voltage Step Sequence]
   Alternating +/- from 2.5V center in 0.1V increments:
     2.6, 2.4, 2.7, 2.3, 2.8, 2.2, ... , 5.0, 0.0
   Total 50 steps (CW/CCW alternating to prevent wire tangling)
   ===================================================================== */
#define   N_STEPS           50

void BuildVoltageSequence(double voltSeq[N_STEPS])
{
    for (int i = 0; i < N_STEPS; i++)
    {
        double delta = 0.1 * (i / 2 + 1);  // 0.1, 0.1, 0.2, 0.2, ..., 2.5, 2.5
        if (i % 2 == 0)
            voltSeq[i] = 2.5 + delta;   // CW
        else
            voltSeq[i] = 2.5 - delta;   // CCW
    }
}

/* =====================================================================
   [CalculateGyroBias]
   Recursive mean:  y_bar_k = (1 - 1/k)*y_bar_{k-1} + (1/k)*y_k,  y_bar_0 = 0
   ===================================================================== */
double CalculateGyroBias(TaskHandle taskAI, int nSamples)
{
    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;
    double  y_bar = 0.0;

    printf("[Gyro Bias Estimation] Collecting %d samples...\n", nSamples);

    for (int k = 1; k <= nSamples; k++)
    {
        DAQmxReadAnalogF64(taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, readArray, 2, &sampsPerChanRead, NULL);

        double y_k = readArray[0];
        y_bar = (1.0 - 1.0 / k) * y_bar + (1.0 / k) * y_k;

        Sleep(5); // 200Hz = 5ms
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

    /* -----------------------------------------------------------------
       DAQ Task handles
    ----------------------------------------------------------------- */
    TaskHandle taskAI = 0;
    TaskHandle taskAO0 = 0;
    TaskHandle taskAO1 = 0;

    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;

    /* -----------------------------------------------------------------
       Create output directory
    ----------------------------------------------------------------- */
    const char* outputDir = "motor_sweep_data";
    _mkdir(outputDir);
    printf("Output folder: %s\n\n", outputDir);

    /* -----------------------------------------------------------------
       Prepare voltage sequence
    ----------------------------------------------------------------- */
    double voltSeq[N_STEPS];
    BuildVoltageSequence(voltSeq);

    /* -----------------------------------------------------------------
       Data buffer (one step)
    ----------------------------------------------------------------- */
    double bufTime[N_HOLD];
    double bufVcmd[N_HOLD];
    double bufVg[N_HOLD];
    double bufPot[N_HOLD];
    double bufOmega[N_HOLD];

    /* =================================================================
       1. Create tasks and configure channels
    ================================================================= */
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO0);
    DAQmxCreateTask("", &taskAO1);

    DAQmxCreateAIVoltageChan(taskAI, "Dev3/ai2, Dev3/ai3", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO0, "Dev3/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev3/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO0);
    DAQmxStartTask(taskAO1);

    /* =================================================================
       2. Initialize: switch OFF, motor stop
    ================================================================= */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // switch OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // motor stop

    printf("============================================================\n");
    printf("  Motor Modeling - Voltage Sweep (%d Steps)\n", N_STEPS);
    printf("  K_gimbal = %.4f [(rad/s)/V]\n", K_GIMBAL);
    printf("============================================================\n\n");

    /* =================================================================
       3. Print voltage sequence
    ================================================================= */
    printf("[Voltage Sequence (Total %d steps)]\n", N_STEPS);
    for (int i = 0; i < N_STEPS; i++)
    {
        printf("  Step %2d: Vcmd = %.1f V  (%s)\n",
            i + 1, voltSeq[i],
            (i % 2 == 0) ? "CW (+)" : "CCW (-)");
    }
    printf("\n");

    /* =================================================================
       4. Gyro bias estimation (motor stopped)
    ================================================================= */
    printf("[Step 0] Gyro bias estimation - confirm motor is stopped, then press any key.\n");
    getchar();

    double Vg_offset = CalculateGyroBias(taskAI, N_BIAS);

    /* =================================================================
       5. Experiment start prompt
    ================================================================= */
    printf("[Step 1] Turn on the gimbal switch, then press any key.\n");
    printf("  * Emergency stop: Spacebar\n\n");
    getchar();

    GetAsyncKeyState(VK_SPACE); // flush buffer

    /* =================================================================
       6. Voltage step loop
    ================================================================= */
    int emergencyStop = 0;

    for (int step = 0; step < N_STEPS && !emergencyStop; step++)
    {
        double      Vcmd = voltSeq[step];
        const char* dir = (step % 2 == 0) ? "CW" : "CCW";

        printf("-----------------------------------------\n");
        printf("[Step %2d/%d]  Vcmd = %.1f V  (%s)  -> hold 10 sec\n",
            step + 1, N_STEPS, Vcmd, dir);

        /* Apply voltage */
        DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 3.0, NULL); // switch ON
        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vcmd, NULL); // motor voltage

        /* Collect samples for 10 seconds */
        time_init_step = GetWindowTime();
        int count = 0;

        while (1)
        {
            /* Emergency stop check */
            if (GetAsyncKeyState(VK_SPACE) & 0x8000)
            {
                printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
                emergencyStop = 1;
                break;
            }

            time_elapsed = (GetWindowTime() - time_init_step) * 0.001; // [sec]

            /* Exit loop after HOLD_TIME seconds */
            if (time_elapsed >= HOLD_TIME) break;

            /* DAQ read */
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

            /* Buffer overflow guard */
            if (count < N_HOLD)
            {
                bufTime[count] = time_elapsed;
                bufVcmd[count] = Vcmd;
                bufVg[count] = Vg;
                bufPot[count] = Vpot;
                bufOmega[count] = omega;
                count++;
            }

            /* Maintain sampling rate (200Hz = 5ms) */
            while (1)
            {
                time_curr = GetWindowTime();
                if (time_curr - time_init_step - (count - 1) * SAMPLING_TIME * 1000.0
                    >= SAMPLING_TIME * 1000.0) break;
            }
        }

        /* -----------------------------------------------------------
           Save to file
        ----------------------------------------------------------- */
        if (count > 0)
        {
            char filename[256];
            sprintf(filename, "%s/step_%02d_V%.1f_%s.out",
                outputDir, step + 1, Vcmd, dir);

            FILE* pFile = fopen(filename, "w+t");
            if (pFile)
            {
                fprintf(pFile, "%% Motor Sweep Step %d/%d\n", step + 1, N_STEPS);
                fprintf(pFile, "%% Vcmd       = %.4f [V]\n", Vcmd);
                fprintf(pFile, "%% Direction  = %s\n", dir);
                fprintf(pFile, "%% Vg_offset  = %.6f [V]\n", Vg_offset);
                fprintf(pFile, "%% K_gimbal   = %.6f [(rad/s)/V]\n\n", K_GIMBAL);
                fprintf(pFile, "Time[s]              Vcmd[V]              Vg_raw[V]            Pot[V]               Omega[rad/s]\n");

                for (int i = 0; i < count; i++)
                {
                    fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f\n",
                        bufTime[i],
                        bufVcmd[i],
                        bufVg[i],
                        bufPot[i],
                        bufOmega[i]);
                }
                fclose(pFile);
                printf("  -> Saved: %s  (%d samples)\n", filename, count);
            }
            else
            {
                printf("  !! Failed to open file: %s\n", filename);
            }
        }

        /* Return motor to neutral and wait 1 sec before next step (prevent wire tangling) */
        if (!emergencyStop && step < N_STEPS - 1)
        {
            DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);
            Sleep(1000);
        }
    }

    /* =================================================================
       7. Shutdown: stop motor, release tasks
    ================================================================= */
    printf("\n============================================================\n");
    printf("  Experiment finished - stopping motor.\n");
    printf("============================================================\n");

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // switch OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // motor stop

    DAQmxStopTask(taskAI);   DAQmxClearTask(taskAI);
    DAQmxStopTask(taskAO0);  DAQmxClearTask(taskAO0);
    DAQmxStopTask(taskAO1);  DAQmxClearTask(taskAO1);

    printf("\n[Done] All data saved to '%s' folder.\n", outputDir);
    printf("Vg_offset = %.6f V,  K_gimbal = %.4f (rad/s)/V\n",
        Vg_offset, K_GIMBAL);
}