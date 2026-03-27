//Hello 
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include "NIDAQmx.h"

double GetWindowTime(void)
{
    LARGE_INTEGER   liEndCounter, liFrequency;

    QueryPerformanceCounter(&liEndCounter);
    QueryPerformanceFrequency(&liFrequency);

    return(liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}; // [ms]

#define   N_STEP          (int)   ( FINAL_TIME*SAMPLING_FREQ )
#define   FINAL_TIME      (double)(            20.0 )
#define   SAMPLING_FREQ      (double)(               200 )
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI         (double)( 3.14159265358979  )

// ------------------------------------------------------------------------------------------

void main(void)
{
    FILE* pFile;
    int32 error;

    double      time_curr = 0.0;
    double      time_init = 0.0;
    double      time = 0.0;
    char      OutFileName[100] = { "" };
    double      Freq = 10.0;

    int         idx = 0;
    int         count = 0;

    unsigned   i = 0;
    double      Standard = 1.0;
    double      OutData[N_STEP] = { 0.0, };
    double      OutTime[N_STEP] = { 0.0, };
    double      OutVcmd2[N_STEP] = { 0.0, }; // [ao1 저장용]
    double      OutVcmd[N_STEP] = { 0.0, };

    double      Vcmd = 0.0;
    double      Vcmd2 = 0.0;      // [ao1 추가] ao1 출력 전압 (ai0 신호 복사본)
    float64      Vin = 0.0;

    TaskHandle   taskAI = 0;
    TaskHandle   taskAO = 0;
    TaskHandle   taskAO1 = 0;     // [ao1 추가] Task 핸들

    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO);
    DAQmxCreateTask("", &taskAO1); // [ao1 추가] Task 생성

    DAQmxCreateAIVoltageChan(taskAI, "Dev2/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO, "Dev2/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev2/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, ""); // [ao1 추가] Dev2/ao1 채널 열기 (0~5V 출력)

    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO);

    double time_test = GetWindowTime();

    printf("Press any key to start the program.... \n");
    getchar();

    time_init = GetWindowTime();
    time_curr = time_init;
    do
    {
        time = (time_curr - time_init) * 0.001;

        /* DAQ Writing : Analog Output */
        Vcmd = Standard + 1.0 * (cos(2.0 * UNIT_PI * Freq * time));
        DAQmxWriteAnalogScalarF64(taskAO, 1.0, 5.0, Vcmd, NULL);    // 아날로그 스케일로 출력 

        /* DAQ Reading */
        error = DAQmxReadAnalogScalarF64(taskAI, 10.0, &Vin, NULL);
        if (error != 0)
        {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("DAQ error: %s\n", errBuff);
        }
        else
        {
            Vin += 2.5;

        }

        /* 3. [ao1 핵심 수정] DAQ Writing : Analog Output 1 (ao1) -> ai0 신호(Vin) 그대로 쏘기 */
        Vcmd2 = Vin; // 방금 읽어서 2.5V 더해진 Vin 값을 Vcmd2에 복사
        DAQmxWriteAnalogScalarF64(taskAO1, 1.0, 5.0, Vcmd2, NULL);

        /* Memory Write */
        OutTime[count] = time;
        OutVcmd[count] = Vcmd;
        OutVcmd2[count] = Vcmd2;      // [ao1] 바이패스된 신호 기록
        OutData[count] = Vin;

        /* check the simulation time and loop count */
        while (1)
        {
            time_curr = GetWindowTime();

            if (time_curr - time_init - count * SAMPLING_TIME * 1000 >= (SAMPLING_TIME * 1000.0)) break;
        }
    } while (count++ < N_STEP - 1);

    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO);
    DAQmxStopTask(taskAO1); // [ao1 추가] Task 정지

    /* Data Print */
    sprintf(OutFileName, "%1.1f", SAMPLING_FREQ);

    pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

    for (idx = 0; idx < N_STEP; idx++)
    {
        // 파일에 Vcmd2(ao1 출력) 값도 같이 저장
        fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f\n", OutTime[idx], OutVcmd[idx], OutVcmd2[idx], OutData[idx]);
    }

    fclose(pFile);
}
