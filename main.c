#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include <direct.h>   // _mkdir
#include "NIDAQmx.h"

/* =====================================================================
   [시간 함수] — 현재 시각을 ms 단위로 반환
   ===================================================================== */
double GetWindowTime(void)
{
    LARGE_INTEGER liCounter, liFrequency;           // [수정1] liEndCounter → liCounter (의미에 맞게)
    QueryPerformanceCounter(&liCounter);
    QueryPerformanceFrequency(&liFrequency);
    return (liCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
} // [ms]

/* =====================================================================
   [매크로 정의]
   ===================================================================== */
#define   SAMPLING_FREQ     (double)( 200.0 )
#define   SAMPLING_TIME     (double)( 1.0 / SAMPLING_FREQ )
#define   HOLD_TIME         (double)( 10.0 )         // 각 전압 유지 시간 [sec]
#define   N_HOLD            (int)   ( HOLD_TIME * SAMPLING_FREQ + 100 )  // [수정2] 여유 버퍼 +100

#define   N_BIAS            (int)   ( 200 )           // bias 추정 샘플 수 (1초)
#define   UNIT_PI           (double)( 3.14159265358979 )

   // -----------------------------------------------------------------------
   // [자이로 Scale Factor]
   //   k_g = 0.67 [mV/(deg/s)]
   //   ω_h [rad/s] = (V_g - V_g_offset) [V] × 1000 [mV/V] / 0.67 [mV/(deg/s)] × (π/180)
   // -----------------------------------------------------------------------
#define   K_GIMBAL          (double)( 1000.0 / 0.67 * UNIT_PI / 180.0 )  // ≈ 26.05 [(rad/s)/V]

/* =====================================================================
   [전압 스텝 시퀀스 생성]
   2.5V 기준으로 ±0.1 교대 증가:
     2.6, 2.4, 2.7, 2.3, 2.8, 2.2, ... , 5.0, 0.0
   총 50스텝 (CW/CCW 교대 → 선 꼬임 방지)
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
   [CalculateGyroBias 함수]
   점화식:  ȳₖ = (1 - 1/k)*ȳₖ₋₁ + (1/k)*yₖ,  ȳ₀ = 0
   ===================================================================== */
double CalculateGyroBias(TaskHandle taskAI, int nSamples)
{
    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;
    double  y_bar = 0.0;

    printf("[Gyro Bias 추정 시작] %d 샘플 수집 중...\n", nSamples);

    for (int k = 1; k <= nSamples; k++)
    {
        DAQmxReadAnalogF64(taskAI, 1, 10.0,
            DAQmx_Val_GroupByChannel,
            readArray, 2, &sampsPerChanRead, NULL);

        double y_k = readArray[0];
        y_bar = (1.0 - 1.0 / k) * y_bar + (1.0 / k) * y_k;

        Sleep(5); // 200Hz = 5ms
    }

    printf("[Gyro Bias 추정 완료] Vg_offset = %.6f V\n\n", y_bar);
    return y_bar;
}

/* =====================================================================
   [main 함수]
   ===================================================================== */
void main(void)
{
    int32   error;
    double  time_curr = 0.0;
    double  time_init_step = 0.0;
    double  time_elapsed = 0.0;

    /* -----------------------------------------------------------------
       DAQ Task 핸들
    ----------------------------------------------------------------- */
    TaskHandle taskAI = 0;
    TaskHandle taskAO0 = 0;
    TaskHandle taskAO1 = 0;

    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;

    /* -----------------------------------------------------------------
       출력 폴더 생성                                [수정3] 특수문자 제거
    ----------------------------------------------------------------- */
    const char* outputDir = "motor_sweep_data";
    _mkdir(outputDir);
    printf("출력 폴더: %s\n\n", outputDir);

    /* -----------------------------------------------------------------
       전압 시퀀스 준비
    ----------------------------------------------------------------- */
    double voltSeq[N_STEPS];
    BuildVoltageSequence(voltSeq);

    /* -----------------------------------------------------------------
       데이터 버퍼 (스텝 1개분)
    ----------------------------------------------------------------- */
    double bufTime[N_HOLD];
    double bufVcmd[N_HOLD];
    double bufVg[N_HOLD];
    double bufPot[N_HOLD];
    double bufOmega[N_HOLD];

    /* =================================================================
       1. Task 생성 및 채널 설정
    ================================================================= */
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO0);
    DAQmxCreateTask("", &taskAO1);

    DAQmxCreateAIVoltageChan(taskAI, "Dev3/ai2, Dev3/ai3", "",
        DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO0, "Dev3/ao0", "",
        0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev3/ao1", "",
        0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO0);
    DAQmxStartTask(taskAO1);

    /* =================================================================
       2. 초기화: 스위치 OFF, 모터 정지
    ================================================================= */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // 스위치 OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // 모터 정지

    printf("============================================================\n");
    printf("  Motor Modeling - Voltage Sweep (%d Steps)\n", N_STEPS);
    printf("  K_gimbal = %.4f [(rad/s)/V]\n", K_GIMBAL);
    printf("============================================================\n\n");

    /* =================================================================
       3. 전압 시퀀스 미리 출력
    ================================================================= */
    printf("[전압 시퀀스 (총 %d스텝)]\n", N_STEPS);
    for (int i = 0; i < N_STEPS; i++)
    {
        printf("  Step %2d: Vcmd = %.1f V  (%s)\n",
            i + 1, voltSeq[i],
            (i % 2 == 0) ? "CW (+)" : "CCW (-)");
    }
    printf("\n");

    /* =================================================================
       4. Gyro Bias 추정 (모터 정지 상태)
    ================================================================= */
    printf("[Step 0] Gyro Bias 추정 - 모터 정지 상태 확인 후 아무 키나 누르세요.\n");
    getchar();

    double Vg_offset = CalculateGyroBias(taskAI, N_BIAS);

    /* =================================================================
       5. 실험 시작 안내
    ================================================================= */
    printf("[Step 1] 짐벌 스위치를 켜고 아무 키나 누르세요.\n");
    printf("※ 긴급 정지: 스페이스바(Spacebar)\n\n");
    getchar();

    GetAsyncKeyState(VK_SPACE); // 버퍼 비우기

    /* =================================================================
       6. 전압 스텝 루프
    ================================================================= */
    int emergencyStop = 0;

    for (int step = 0; step < N_STEPS && !emergencyStop; step++)
    {
        double      Vcmd = voltSeq[step];
        const char* dir = (step % 2 == 0) ? "CW" : "CCW";

        printf("-----------------------------------------\n");
        printf("[Step %2d/%d]  Vcmd = %.1f V  (%s)  -> 10초 유지\n",
            step + 1, N_STEPS, Vcmd, dir);

        /* 해당 전압 인가 */
        DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 3.0, NULL); // 스위치 ON
        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vcmd, NULL); // 모터 전압

        /* 10초 동안 샘플 수집 */
        time_init_step = GetWindowTime();
        int count = 0;

        while (1)                                           // [수정4] do-while → while(1)
        {
            /* 긴급 정지 확인 */
            if (GetAsyncKeyState(VK_SPACE) & 0x8000)
            {
                printf("\n[긴급 정지] 스페이스바 입력!\n");
                emergencyStop = 1;
                break;
            }

            time_elapsed = (GetWindowTime() - time_init_step) * 0.001; // [sec]

            /* [수정5] 시간 기반 종료 — 10초 경과 시 루프 탈출 */
            if (time_elapsed >= HOLD_TIME) break;

            /* DAQ 읽기 */
            error = DAQmxReadAnalogF64(taskAI, 1, 10.0,
                DAQmx_Val_GroupByChannel,
                readArray, 2, &sampsPerChanRead, NULL);

            if (error != 0)
            {
                char errBuff[2048];
                DAQmxGetExtendedErrorInfo(errBuff, 2048);
                printf("DAQ 읽기 에러: %s\n", errBuff);
            }

            double Vg = readArray[0];
            double Vpot = readArray[1];
            double omega = K_GIMBAL * (Vg - Vg_offset);

            /* 버퍼 오버플로우 방지 */          // [수정6] 버퍼 범위 체크 추가
            if (count < N_HOLD)
            {
                bufTime[count] = time_elapsed;
                bufVcmd[count] = Vcmd;
                bufVg[count] = Vg;
                bufPot[count] = Vpot;
                bufOmega[count] = omega;
                count++;
            }

            /* 샘플링 타임 유지 (200Hz = 5ms) */
            while (1)
            {
                time_curr = GetWindowTime();
                if (time_curr - time_init_step - (count - 1) * SAMPLING_TIME * 1000.0
                    >= SAMPLING_TIME * 1000.0) break;
            }
        }

        /* -----------------------------------------------------------
           파일 저장                              [수정7] 조건 count > 0 으로 단순화
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
                printf("  -> 저장 완료: %s  (%d 샘플)\n", filename, count);
            }
            else
            {
                printf("  !! 파일 열기 실패: %s\n", filename);
            }
        }

        /* 다음 스텝 전에 모터를 정지 위치로 복귀 후 1초 대기 (선 꼬임 방지) */
        if (!emergencyStop && step < N_STEPS - 1)
        {
            DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);
            Sleep(1000);
        }
    }

    /* =================================================================
       7. 종료: 모터 정지, Task 해제
    ================================================================= */
    printf("\n============================================================\n");
    printf("  실험 종료 - 모터 정지합니다.\n");
    printf("============================================================\n");

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // 스위치 OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // 모터 정지

    DAQmxStopTask(taskAI);   DAQmxClearTask(taskAI);
    DAQmxStopTask(taskAO0);  DAQmxClearTask(taskAO0);
    DAQmxStopTask(taskAO1);  DAQmxClearTask(taskAO1);

    printf("\n[완료] 모든 데이터가 '%s' 폴더에 저장되었습니다.\n", outputDir);
    printf("Vg_offset = %.6f V,  K_gimbal = %.4f (rad/s)/V\n",
        Vg_offset, K_GIMBAL);
    printf("총 %d 스텝 실행됨.\n", emergencyStop ? step : N_STEPS);
}
