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

#define   N_STEP            (int)   ( FINAL_TIME*SAMPLING_FREQ )
#define   FINAL_TIME        (double)( 20.0 )
#define   SAMPLING_FREQ     (double)( 200 )
#define   SAMPLING_TIME     (double)( 1.0/SAMPLING_FREQ )
#define   UNIT_PI           (double)( 3.14159265358979 )

void main(void)
{
    FILE* pFile;
    int32 error;

    double time_curr = 0.0;
    double time_init = 0.0;
    double time = 0.0;
    char OutFileName[100] = { "" };

    // [제어 파라미터]
    double Freq = 10.0;          // 사인파 주파수
    double Amplitude = 1;     // 사인파 진폭

    int idx = 0;
    int count = 0;

    // [데이터 저장용 배열]
    double OutTime[N_STEP] = { 0.0, };
    double OutAO0[N_STEP] = { 0.0, };
    double OutAO1[N_STEP] = { 0.0, };
    double OutAI2[N_STEP] = { 0.0, };
    double OutAI3[N_STEP] = { 0.0, };

    double Vcmd_ao0 = 0.0;
    double Vcmd_ao1 = 0.0;

    float64 readArray[2] = { 0.0, 0.0 };
    int32 sampsPerChanRead;

    TaskHandle taskAI = 0;
    TaskHandle taskAO0 = 0;
    TaskHandle taskAO1 = 0;

    // 1. Task 생성
    DAQmxCreateTask("", &taskAI);
    DAQmxCreateTask("", &taskAO0);
    DAQmxCreateTask("", &taskAO1);

    // 2. 채널 설정
    DAQmxCreateAIVoltageChan(taskAI, "Dev3/ai2, Dev3/ai3", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO0, "Dev3/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(taskAO1, "Dev3/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

    // 3. Task 시작
    DAQmxStartTask(taskAI);
    DAQmxStartTask(taskAO0);
    DAQmxStartTask(taskAO1);

    // [초기화 셋업] 
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // 스위치 OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // 정지

    printf("초기화 완료 (AO0: 0V, AO1: 2.5V).\n");
    printf("프로그램을 시작하고 짐벌 스위치를 켜려면 아무 키나 누르세요...\n");
    printf("※ 구동 중 긴급 정지하려면 '스페이스바(Spacebar)'를 누르세요.\n");
    getchar();

    // 시작 전 키보드 버퍼 비우기 (오작동 방지)
    GetAsyncKeyState(VK_SPACE);

    time_init = GetWindowTime();
    time_curr = time_init;

    // 제어 루프 시작
    do
    {
        time = (time_curr - time_init) * 0.001; // [sec]

        /* -------------------------------------------------------------
           [긴급 정지 확인] : 스페이스바 입력 감지
        --------------------------------------------------------------*/
        if (GetAsyncKeyState(VK_SPACE) & 0x8000)
        {
            printf("\n[긴급 정지] 스페이스바 입력 감지! 제어 루프를 즉시 종료합니다.\n");
            break; // 루프 탈출
        }

        /* -------------------------------------------------------------
           [DAQ Writing]
        --------------------------------------------------------------*/
        Vcmd_ao0 = 3.0;
        DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, Vcmd_ao0, NULL);

        Vcmd_ao1 = 2.5 + Amplitude * sin(2.0 * UNIT_PI * Freq * time);
        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vcmd_ao1, NULL);

        /* -------------------------------------------------------------
           [DAQ Reading]
        --------------------------------------------------------------*/
        error = DAQmxReadAnalogF64(taskAI, 1, 10.0, DAQmx_Val_GroupByChannel, readArray, 2, &sampsPerChanRead, NULL);
        if (error != 0)
        {
            char errBuff[2048];
            DAQmxGetExtendedErrorInfo(errBuff, 2048);
            printf("DAQ 읽기 에러: %s\n", errBuff);
        }

        /* -------------------------------------------------------------
           [Data Logging]
        --------------------------------------------------------------*/
        OutTime[count] = time;
        OutAO0[count] = Vcmd_ao0;
        OutAO1[count] = Vcmd_ao1;
        OutAI2[count] = readArray[0];
        OutAI3[count] = readArray[1];

        /* 샘플링 타임 유지 및 루프 체크 */
        while (1)
        {
            time_curr = GetWindowTime();
            if (time_curr - time_init - count * SAMPLING_TIME * 1000 >= (SAMPLING_TIME * 1000.0)) break;
        }
    } while (count++ < N_STEP - 1);

    // [종료 시퀀스] 로봇 정지를 위해 전압 초기화 (정상 종료 및 비상 정지 모두 적용)
    printf("\n로봇을 정지 위치(AO0: 0V, AO1: 2.5V)로 복귀시킵니다...\n");
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL); // 스위치 OFF
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL); // 정지

    // Task 정지 및 자원 해제
    DAQmxStopTask(taskAI);
    DAQmxStopTask(taskAO0);
    DAQmxStopTask(taskAO1);

    DAQmxClearTask(taskAI);
    DAQmxClearTask(taskAO0);
    DAQmxClearTask(taskAO1);

    /* 파일 저장 */
    sprintf(OutFileName, "%1.1f", SAMPLING_FREQ);
    pFile = fopen(strcat(OutFileName, "_data.out"), "w+t");

    fprintf(pFile, "Time[s]\t\tAO0(Switch)\tAO1(Sine)\tAI2(Read)\tAI3(Read)\n");

    // [수정] 긴급 정지 시 count가 N_STEP보다 작으므로, 실제로 기록된 부분(count)까지만 저장합니다.
    for (idx = 0; idx < count; idx++)
    {
        fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f\n",
            OutTime[idx], OutAO0[idx], OutAO1[idx], OutAI2[idx], OutAI3[idx]);
    }
    fclose(pFile);

    printf("제어 완료 및 데이터 저장 성공. (저장된 데이터 개수: %d)\n", count);
}