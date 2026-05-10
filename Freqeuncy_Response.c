// =====================================================================
//  motor_freq_response.c
//  Motor Frequency Response Measurement (Bode Plot)
//  각 주파수마다 사인파 입력 → DFT로 gain/phase 계산
//
//  [변경사항]
//  - SINE_AMP       : 1.0 → 0.45 V
//  - 주파수 범위    : 0.1 ~ 15 Hz (12포인트, log-scale)
//  - N_CYCLES       : 저주파 실험 시간 과다 방지를 위해 adaptive하게 변경
//  - [버그수정] DFT 입력을 Vc → Vcmd 로 수정 (Vcmd→ω 전달함수 측정이 목적)
// =====================================================================
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <windows.h>
#include <direct.h>
#include "NIDAQmx.h"

/* =====================================================================
   [Time Function]
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
#define UNIT_PI         (double)(3.14159265358979)
#define K_GIMBAL        (double)(1000.0 / 0.67 * UNIT_PI / 180.0)

// --- 사인파 파라미터 ---
// [수정] 1.0V → 0.45V  (dead zone 상단 V1 기준, D*V1 하한 충족)
#define SINE_AMP        (double)(0.45)

// --- Adaptive 사이클 수 설정 ---
// 저주파에서 실험 시간이 너무 길어지는 것을 방지
// f < 0.5Hz  → 5 cycles (최대 50s/freq)
// f < 2.0Hz  → 10 cycles
// f >= 2.0Hz → 20 cycles
#define N_CYCLES_LOW    (int)(5)
#define N_CYCLES_MID    (int)(10)
#define N_CYCLES_HIGH   (int)(20)
#define FREQ_THR_LOW    (double)(0.5)
#define FREQ_THR_MID    (double)(2.0)

// 과도구간 제외: 항상 3사이클
#define N_SKIP_CYCLES   (int)(3)

// --- 테스트 주파수 목록 (0.1 ~ 15 Hz, 12포인트 log-scale) ---
// [수정] 기존 9포인트(0.5~20Hz) → 12포인트(0.1~15Hz)
#define N_FREQS         12
static const double TEST_FREQS[N_FREQS] = {
    0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0
};

// --- LUT 파라미터 ---
#define K_LIN       (double)(8.1278)
#define P_CW_A      (double)(-2.6219)
#define P_CW_B      (double)(30.4498)
#define P_CW_C      (double)(-63.4266)
#define P_CCW_A     (double)(2.3946)
#define P_CCW_B     (double)(4.7513)
#define P_CCW_C     (double)(-23.2939)
#define DEAD_THRESH (double)(0.5)

#define N_BIAS      (int)(200)

/* =====================================================================
   [GetNCycles] - 주파수에 따라 사이클 수 반환
   ===================================================================== */
int GetNCycles(double freq)
{
    if      (freq < FREQ_THR_LOW) return N_CYCLES_LOW;
    else if (freq < FREQ_THR_MID) return N_CYCLES_MID;
    else                          return N_CYCLES_HIGH;
}

/* =====================================================================
   [InverseMap]
   ===================================================================== */
double InverseMap(double vcmd_ref)
{
    double omega_target = K_LIN * vcmd_ref;
    double Vc = 2.5;

    if (omega_target > DEAD_THRESH)
    {
        double a = P_CW_A, b = P_CW_B, c = P_CW_C - omega_target;
        double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int r1ok = (r1 >= 2.5) && (r1 <= 5.0);
            int r2ok = (r2 >= 2.5) && (r2 <= 5.0);
            if      (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }
    else if (omega_target < -DEAD_THRESH)
    {
        double a = P_CCW_A, b = P_CCW_B, c = P_CCW_C - omega_target;
        double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int r1ok = (r1 >= 0.0) && (r1 <= 2.5);
            int r2ok = (r2 >= 0.0) && (r2 <= 2.5);
            if      (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }

    if (Vc < 0.0) Vc = 0.0;
    if (Vc > 5.0) Vc = 5.0;
    return Vc;
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
   [ComputeGainPhase]

   DFT를 이용해 특정 주파수에서의 gain/phase 계산

   원리:
     입력 x(t) = Vcmd = A*sin(wt)
     출력 y(t) = omega = B*sin(wt + phi)

     DFT로 주파수 w에서의 복소수 성분을 추출:
       X = (2/N) * Σ x[k] * e^(-jwk*dt)
       Y = (2/N) * Σ y[k] * e^(-jwk*dt)

     gain  = |Y| / |X|
     phase = angle(Y) - angle(X)   [rad]

   [버그수정] 입력 버퍼를 Vc → Vcmd로 변경
             (측정 대상 전달함수: Vcmd → omega)
   ===================================================================== */
void ComputeGainPhase(
    double* vcmd_buf,    // [수정] 입력 버퍼: Vcmd (선형화된 명령 전압)
    double* omega_buf,   // 출력 버퍼: omega [rad/s]
    int     N,
    double  freq,
    double  dt,
    double* gain_out,
    double* phase_out
)
{
    double w = 2.0 * UNIT_PI * freq;
    double re_x = 0.0, im_x = 0.0;
    double re_y = 0.0, im_y = 0.0;

    for (int k = 0; k < N; k++) {
        double t = k * dt;
        double cos_wt = cos(w * t);
        double sin_wt = sin(w * t);

        re_x += vcmd_buf[k]  * cos_wt;
        im_x -= vcmd_buf[k]  * sin_wt;
        re_y += omega_buf[k] * cos_wt;
        im_y -= omega_buf[k] * sin_wt;
    }

    re_x *= (2.0 / N);  im_x *= (2.0 / N);
    re_y *= (2.0 / N);  im_y *= (2.0 / N);

    double mag_x = sqrt(re_x * re_x + im_x * im_x);
    double mag_y = sqrt(re_y * re_y + im_y * im_y);

    *gain_out  = (mag_x > 1e-9) ? (mag_y / mag_x) : 0.0;
    *phase_out = (atan2(im_y, re_y) - atan2(im_x, re_x)) * 180.0 / UNIT_PI;

    while (*phase_out >  180.0) *phase_out -= 360.0;
    while (*phase_out < -180.0) *phase_out += 360.0;
}

/* =====================================================================
   [main]
   ===================================================================== */
void main(void)
{
    int32  error;
    TaskHandle taskAI = 0, taskAO0 = 0, taskAO1 = 0;
    float64 readArray[2] = { 0.0, 0.0 };
    int32   sampsPerChanRead;

    const char* outputDir = "bode_data";
    _mkdir(outputDir);

    double result_freq [N_FREQS] = {0};
    double result_gain [N_FREQS] = {0};
    double result_phase[N_FREQS] = {0};

    /* ---------------------------------------------------------------
       DAQ Task 생성
    --------------------------------------------------------------- */
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

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    printf("============================================================\n");
    printf("  Motor Frequency Response (Bode Plot)\n");
    printf("  Sine amplitude : %.2f V (Vcmd)\n", SINE_AMP);
    printf("  Cycles/freq    : adaptive (low<%.1f: %d, mid<%.1f: %d, high: %d)\n",
           FREQ_THR_LOW, N_CYCLES_LOW, FREQ_THR_MID, N_CYCLES_MID, N_CYCLES_HIGH);
    printf("  Skip cycles    : %d\n", N_SKIP_CYCLES);
    printf("  Fs             : %.0f Hz\n", SAMPLING_FREQ);
    printf("  Frequencies    : ");
    for (int i = 0; i < N_FREQS; i++) printf("%.1f ", TEST_FREQS[i]);
    printf("Hz\n");

    // 예상 총 실험 시간 출력
    double total_est = 0.0;
    for (int i = 0; i < N_FREQS; i++)
        total_est += (1.0 / TEST_FREQS[i]) * GetNCycles(TEST_FREQS[i]);
    printf("  예상 총 실험 시간 (대기 제외): ~%.0f s (%.1f min)\n", total_est, total_est/60.0);
    printf("============================================================\n\n");

    /* ---------------------------------------------------------------
       Gyro Bias
    --------------------------------------------------------------- */
    printf("[Step 0] Stop motor, then press Enter.\n");
    getchar();
    double Vg_offset = CalculateGyroBias(taskAI, N_BIAS);

    printf("[Step 1] Turn on gimbal switch, then press Enter.\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 3.0, NULL);

    /* ---------------------------------------------------------------
       주파수별 실험 루프
    --------------------------------------------------------------- */
    for (int fi = 0; fi < N_FREQS; fi++)
    {
        double freq     = TEST_FREQS[fi];
        int    n_cycles = GetNCycles(freq);               // [수정] adaptive
        double T_period = 1.0 / freq;
        double T_total  = T_period * n_cycles;
        double T_skip   = T_period * N_SKIP_CYCLES;
        int    N_total  = (int)(T_total  * SAMPLING_FREQ);
        int    N_skip   = (int)(T_skip   * SAMPLING_FREQ);
        int    N_valid  = N_total - N_skip;

        // N_skip이 N_total을 초과하지 않도록 보호
        if (N_skip >= N_total) {
            printf("[WARN] f=%.1f Hz: N_skip >= N_total, skipping.\n", freq);
            continue;
        }

        printf("------------------------------------------------------------\n");
        printf("[%d/%d] f = %.1f Hz  |  cycles = %d  |  T_total = %.1f s  |  N_valid = %d\n",
               fi + 1, N_FREQS, freq, n_cycles, T_total, N_valid);

        double* buf_vc    = (double*)malloc(N_total * sizeof(double));
        double* buf_omega = (double*)malloc(N_total * sizeof(double));
        double* buf_time  = (double*)malloc(N_total * sizeof(double));
        double* buf_vcmd  = (double*)malloc(N_total * sizeof(double));

        if (!buf_vc || !buf_omega || !buf_time || !buf_vcmd) {
            printf("[ERROR] malloc failed\n");
            break;
        }

        /* ── 사인파 출력 및 데이터 수집 ── */
        double time_start = GetWindowTime();
        int emergStop = 0;

        for (int k = 0; k < N_total; k++)
        {
            if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
                printf("\n[EMERGENCY STOP]\n");
                emergStop = 1;
                break;
            }

            double t    = k * SAMPLING_TIME;
            double vcmd = SINE_AMP * sin(2.0 * UNIT_PI * freq * t);
            double Vc   = InverseMap(vcmd);

            DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, Vc, NULL);

            error = DAQmxReadAnalogF64(taskAI, 1, 10.0,
                DAQmx_Val_GroupByChannel, readArray, 2,
                &sampsPerChanRead, NULL);

            double Vg    = readArray[0];
            double omega = K_GIMBAL * (Vg - Vg_offset);

            buf_time [k] = t;
            buf_vcmd [k] = vcmd;   // [수정] Vcmd 저장
            buf_vc   [k] = Vc;
            buf_omega[k] = omega;

            while (1) {
                double t_now = (GetWindowTime() - time_start) * 0.001;
                if (t_now - k * SAMPLING_TIME >= SAMPLING_TIME) break;
            }
        }

        DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

        if (emergStop) {
            free(buf_vc); free(buf_omega);
            free(buf_time); free(buf_vcmd);
            break;
        }

        /* ── 과도구간 제외 후 DFT ── */
        // [버그수정] buf_vc_valid → buf_vcmd_valid (Vcmd → omega 전달함수)
        double* buf_vcmd_valid  = buf_vcmd  + N_skip;
        double* buf_omega_valid = buf_omega + N_skip;

        double gain  = 0.0;
        double phase = 0.0;
        ComputeGainPhase(buf_vcmd_valid, buf_omega_valid, N_valid,
                         freq, SAMPLING_TIME, &gain, &phase);

        double gain_dB = 20.0 * log10(gain > 1e-9 ? gain : 1e-9);

        result_freq [fi] = freq;
        result_gain [fi] = gain_dB;
        result_phase[fi] = phase;

        printf("  → Gain = %.4f (%.2f dB)  |  Phase = %.2f deg\n",
               gain, gain_dB, phase);

        /* ── 원시 데이터 저장 ── */
        char fname[256];
        sprintf(fname, "%s/raw_f%.2fHz.out", outputDir, freq);
        FILE* fp = fopen(fname, "w+t");
        if (fp) {
            fprintf(fp, "%% f = %.2f Hz  gain = %.4f (%.2f dB)  phase = %.2f deg\n",
                    freq, gain, gain_dB, phase);
            fprintf(fp, "Time[s]              Vcmd[V]              "
                        "Vc[V]                Omega[rad/s]\n");
            for (int k = 0; k < N_total; k++)
                fprintf(fp, "%20.10f %20.10f %20.10f %20.10f\n",
                        buf_time[k], buf_vcmd[k], buf_vc[k], buf_omega[k]);
            fclose(fp);
        }

        free(buf_vc); free(buf_omega);
        free(buf_time); free(buf_vcmd);

        Sleep(1000);
    }

    /* ---------------------------------------------------------------
       Bode plot 결과 저장
    --------------------------------------------------------------- */
    char fname_bode[256];
    sprintf(fname_bode, "%s/bode_result.out", outputDir);
    FILE* fp_bode = fopen(fname_bode, "w+t");
    if (fp_bode) {
        fprintf(fp_bode, "%% Bode Plot Result\n");
        fprintf(fp_bode, "%% Sine amplitude = %.2f V (Vcmd)\n", SINE_AMP);
        fprintf(fp_bode, "%% Cycles/freq    : adaptive\n");
        fprintf(fp_bode, "%% Input  : Vcmd [V]\n");
        fprintf(fp_bode, "%% Output : Omega [rad/s]\n");
        fprintf(fp_bode, "Freq[Hz]             Gain[dB]             Phase[deg]\n");
        for (int i = 0; i < N_FREQS; i++)
            fprintf(fp_bode, "%20.10f %20.10f %20.10f\n",
                    result_freq[i], result_gain[i], result_phase[i]);
        fclose(fp_bode);
        printf("\n[Saved] %s\n", fname_bode);
    }

    /* ---------------------------------------------------------------
       Shutdown
    --------------------------------------------------------------- */
    DAQmxWriteAnalogScalarF64(taskAO0, 1, 10.0, 0.0, NULL);
    DAQmxWriteAnalogScalarF64(taskAO1, 1, 10.0, 2.5, NULL);

    DAQmxStopTask(taskAI);  DAQmxClearTask(taskAI);
    DAQmxStopTask(taskAO0); DAQmxClearTask(taskAO0);
    DAQmxStopTask(taskAO1); DAQmxClearTask(taskAO1);

    printf("\n[Done] Output folder : '%s/'\n", outputDir);
    printf("\nPress Enter to exit.\n");
    getchar();
}
