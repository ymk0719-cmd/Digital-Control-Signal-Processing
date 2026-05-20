#include "NIDAQmx.h"
#include "_macro.h"

TaskHandle g_taskAI = 0;
TaskHandle g_taskAO = 0;
int32 sampsPerChanRead;
int32 sampsPerChanWritten;

double TEST_FREQS[N_FREQS] = {
    0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0
};

double time_init = 0.0;
double time_elapsed = 0.0;
double Vg_offset = 0.0;
double Vcmd = 0.0;
double Vc = 0.0;
double Vg = 0.0;
double Vpot = 0.0;
double omega = 0.0;
double omega_target = 0.0;
double readArr[2] = { 0.0 };
double writeArr[2] = { 0.0 };
int    nStep = 0;
int    count = 0;


double voltSeq[N_STEPS_MAX] = { 0 };

double buftime[BUF_SIZE];
double bufVcmd[BUF_SIZE];
double bufVc[BUF_SIZE];
double bufVg[BUF_SIZE];
double bufVpot[BUF_SIZE];
double bufomega[BUF_SIZE];
double bufomega_target[BUF_SIZE];


double GetWindowTime(void);
void   memorySet(void);
void   BusyWait_ms(double ms);
void   WaitNextSample(void);
int    IsEmergencyStop(void);

void   DAQ_Init(void);
void   DAQ_Cleanup(void);
void   DAQ_ReadSample(void);
void   motor_power(float64 onoff, double voltage);

double CalculateGyroBias(int nSamples);
int    BuildVoltageSequence(void);
void   RunSweep(void);
double InverseMap(double vcmd_ref);
double Triangle_cmd(double t);
void   RunWaveVerify(int mode);

void   RunBode(void);
static int GetNCycles(double freq);


void main(void)
{
    int mode = 0;

    printf("Press [Enter] to start the program.... \n\n");
    getchar();

    printf("\n[DAQ Initializing...]\n");
    DAQ_Init();
    motor_power(ON, NEUTRAL);

    printf("[Step 0] Confirm motor is stopped, then press [Enter].\n");
    getchar();

    Vg_offset = CalculateGyroBias(N_BIAS);

    do {
        printf("============================================================\n");
        printf("  Hello, are you ready?\n");
        printf("============================================================\n");
        printf("  1. Voltage Sweep\n");
        printf("  2. Triangle Wave Verify\n");
        printf("  3. Sine Wave Verify\n");
        printf("  4. Frequency Response (Bode Plot)\n");
        printf("  0. Exit\n");
        printf("============================================================\n");
        printf("Select mode > ");

        scanf("%d", &mode);
        while (getchar() != '\n');

        switch (mode)
        {
        case EXIT:           break;
        case VOLTAGE_SWEEP:  RunSweep();               break;
        case TRI_VALIDATION: RunWaveVerify(MODE_TRI);  break;
        case SINE_VALIDATION:RunWaveVerify(MODE_SINE); break;
        case FREQ_SWEEP:     RunBode();                break;
        default: printf("[Error] Invalid mode: %d\n", mode); break;
        }
    } while (mode >= 1 && mode <= 4);

    printf("[DAQ Cleaning up...]\n");
    DAQ_Cleanup();
    printf("\nProgram finished. Press [Enter] to exit.\n");
    getchar();
}


double GetWindowTime(void)
{
    LARGE_INTEGER liCounter, liFrequency;
    QueryPerformanceCounter(&liCounter);
    QueryPerformanceFrequency(&liFrequency);
    return (liCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}

void memorySet(void)
{
    memset(buftime, 0, sizeof(buftime));
    memset(bufVcmd, 0, sizeof(bufVcmd));
    memset(bufVg, 0, sizeof(bufVg));
    memset(bufVc, 0, sizeof(bufVc));
    memset(bufVpot, 0, sizeof(bufVpot));
    memset(bufomega, 0, sizeof(bufomega));
    memset(bufomega_target, 0, sizeof(bufomega_target));
}

void BusyWait_ms(double ms)
{
    double time_start = GetWindowTime();
    while (GetWindowTime() - time_start < ms);
}

void WaitNextSample(void)
{
    double target_ms = count * SAMPLING_TIME * 1000.0;
    while (GetWindowTime() - time_init < target_ms);
}

int IsEmergencyStop(void)
{
    return (GetAsyncKeyState(VK_SPACE) & 0x8000) ? 1 : 0;
}


void DAQ_Init(void)
{
    DAQmxCreateTask("", &g_taskAI);
    DAQmxCreateTask("", &g_taskAO);

    DAQmxCreateAIVoltageChan(g_taskAI, DAQ_DEV "/ai2, " DAQ_DEV "/ai3", "",
        DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
    DAQmxCreateAOVoltageChan(g_taskAO, DAQ_DEV "/ao0, " DAQ_DEV "/ao1", "",
        0.0, 5.0, DAQmx_Val_Volts, "");

    DAQmxStartTask(g_taskAI);
    DAQmxStartTask(g_taskAO);
}

void DAQ_Cleanup(void)
{
    motor_power(OFF, NEUTRAL);

    DAQmxStopTask(g_taskAI); DAQmxClearTask(g_taskAI);
    DAQmxStopTask(g_taskAO); DAQmxClearTask(g_taskAO);
}

void DAQ_ReadSample(void)
{
    memset(readArr, 0, sizeof(readArr));
    Vg = 0; Vpot = 0; omega = 0;

    int32 err = READ_DATA(readArr); //error=0 --> success, error>0 -->warning, error<0 --> fail
    if (err != 0) {
        char errBuff[2048];
        DAQmxGetExtendedErrorInfo(errBuff, 2048);
        printf("[DAQ Read Error] %s\n", errBuff);
    }
    Vg = readArr[0];
    Vpot = readArr[1];
    omega = K_GIMBAL * (Vg - Vg_offset);
}

void motor_power(float64 onoff, double voltage)
{
    writeArr[0] = onoff;
    writeArr[1] = (float64)voltage;
    DAQmxWriteAnalogF64(g_taskAO, 1, 1, 10.0,
        DAQmx_Val_GroupByChannel,
        writeArr, &sampsPerChanWritten, NULL);
}


double CalculateGyroBias(int nSamples)
{
    double y_bar = 0.0, y_k = 0.0;
    memset(readArr, 0, sizeof(readArr));

    printf("[Gyro Bias] Collecting %d samples...\n", nSamples);
    for (int k = 1; k <= nSamples; k++) {
        READ_DATA(readArr);
        y_k = readArr[0];
        y_bar = (1.0 - 1.0 / k) * y_bar + (1.0 / k) * y_k;
        BusyWait_ms(5.0);
    }
    printf("[Gyro Bias Done] Vg_offset = %.6f V\n\n", y_bar);
    return y_bar;
}


int BuildVoltageSequence(void)
{
    double deltas[N_STEPS_MAX] = { 0.0 };
    int    nDeltas = 0;
    double vCW = 0.0, vCCW = 0.0;

    for (int i = 1; i <= 50; i++) deltas[nDeltas++] = i * 0.01;
    for (int i = 1; i <= 39; i++) deltas[nDeltas++] = 0.50 + i * 0.05;
    deltas[nDeltas++] = 2.50;

    int nSteps = 0;
    for (int i = 0; i < nDeltas; i++) {
        vCW = 2.5 + deltas[i]; if (vCW > 5.0) vCW = 5.0;
        vCCW = 2.5 - deltas[i]; if (vCCW < 0.0) vCCW = 0.0;
        voltSeq[nSteps++] = vCW;
        voltSeq[nSteps++] = vCCW;
    }
    return nSteps;
}


double InverseMap(double vcmd_ref)
{
    omega_target = K_LIN * vcmd_ref;
    double Vc = 2.5;

    if (omega_target > DEAD_THRESH)
    {
        double a = CW_C2, b = CW_C1, c = CW_C0 - omega_target;
        double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int r1ok = (r1 >= 2.5) && (r1 <= 5.0);
            int r2ok = (r2 >= 2.5) && (r2 <= 5.0);
            if (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }
    else if (omega_target < -DEAD_THRESH)
    {
        double a = CCW_C2, b = CCW_C1, c = CCW_C0 - omega_target;
        double disc = b * b - 4.0 * a * c;
        if (disc >= 0.0) {
            double r1 = (-b + sqrt(disc)) / (2.0 * a);
            double r2 = (-b - sqrt(disc)) / (2.0 * a);
            int r1ok = (r1 >= 0.0) && (r1 <= 2.5);
            int r2ok = (r2 >= 0.0) && (r2 <= 2.5);
            if (r1ok && r2ok) Vc = (r1 < r2) ? r1 : r2;
            else if (r1ok)         Vc = r1;
            else if (r2ok)         Vc = r2;
            else                   Vc = 2.5;
        }
    }

    if (Vc < 0.0) Vc = 0.0;
    if (Vc > 5.0) Vc = 5.0;
    return Vc;
}


double Triangle_cmd(double t)
{
    double phase = fmod(t, TRI_PERIOD) / TRI_PERIOD;
    if (phase < 0.25) return  TRI_AMP * (4.0 * phase);
    else if (phase < 0.75) return  TRI_AMP * (2.0 - 4.0 * phase);
    else                   return  TRI_AMP * (4.0 * phase - 4.0);
}


void RunSweep(void)
{
    const char* dir;
    char  filename[256];
    char* outputDir = "motor_sweep_data";

    nStep = BuildVoltageSequence();
    _mkdir(outputDir);

    printf("============================================================\n");
    printf("  [MODE 1] Voltage Sweep (%d steps)\n", nStep);
    printf("  Deadzone 2.0~3.0V -> 0.01V step\n");
    printf("  Outer    0.0~2.0V, 3.0~5.0V -> 0.05V step\n");
    printf("  Hold time : %.1f sec,  K_gimbal = %.4f (rad/s)/V\n", HOLD_TIME, K_GIMBAL);
    printf("============================================================\n\n");
    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    printf("  * Emergency stop: Spacebar\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    for (int step = 0; step < nStep && !IsEmergencyStop(); step++)
    {
        Vcmd = voltSeq[step];
        dir = (step % 2 == 0) ? "CW" : "CCW";

        printf("-----------------------------------------\n");
        printf("[Step %3d/%d]  Vcmd = %.2f V  (%s)\n", step + 1, nStep, Vcmd, dir);

        motor_power(ON, Vcmd);
        memorySet();
        time_init = GetWindowTime();
        count = 0;

        do
        {
            DAQ_ReadSample();
            time_elapsed = (GetWindowTime() - time_init) * 0.001;

            if (count < N_HOLD) {
                buftime[count] = time_elapsed;
                bufVcmd[count] = Vcmd;
                bufVg[count] = Vg;
                bufVpot[count] = Vpot;
                bufomega[count] = omega;
            }

            count++;
            WaitNextSample();

        } while (!IsEmergencyStop() && time_elapsed < HOLD_TIME);

        if (IsEmergencyStop()) {
            printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
            break;
        }

        if (count > 0) {
            sprintf(filename, "%s/step_%03d_V%.2f_%s.out", outputDir, step + 1, Vcmd, dir);
            FILE* pFile = fopen(filename, "w+t");
            if (pFile) {
                fprintf(pFile, "%% Sweep Step %d/%d  Vcmd=%.4fV  %s\n", step + 1, N_STEPS_MAX, Vcmd, dir);
                fprintf(pFile, "%% Vg_offset=%.6fV  K_gimbal=%.6f\n\n", Vg_offset, K_GIMBAL);
                fprintf(pFile, "Time[s]              Vcmd[V]              Vg_raw[V]            Pot[V]               Omega[rad/s]\n");
                for (int i = 0; i < count; i++)
                    fprintf(pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f\n",
                        buftime[i], bufVcmd[i], bufVg[i], bufVpot[i], bufomega[i]);
                fclose(pFile);
                printf("  -> Saved: %s  (%d samples)\n", filename, count);
            }
            else {
                printf("  !! File open failed: %s\n", filename);
            }
        }

        if (!IsEmergencyStop() && step < nStep - 1) {
            motor_power(ON, NEUTRAL);
            BusyWait_ms(1000.0);
        }
    }

    motor_power(ON, NEUTRAL);
    printf("\n[MODE 1 Done] Output folder: %s\n\n", outputDir);
}

void RunWaveVerify(int mode)
{
    double T_total = (mode == MODE_SINE) ? SINE_T_TOTAL : TRI_T_TOTAL;
    const char* outputDir = (mode == MODE_SINE) ? "sine_verify_data" : "triangle_verify_data";
    char   filename[256];
    memorySet();
    _mkdir(outputDir);

    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n");
    getchar();
    motor_power(ON, NEUTRAL);
    GetAsyncKeyState(VK_SPACE);

    time_init = GetWindowTime();
    time_elapsed = 0.0;
    count = 0;

    do
    {
        DAQ_ReadSample();
        time_elapsed = (GetWindowTime() - time_init) * 0.001;

        Vcmd = (mode == MODE_SINE) ? SINE_CMD(time_elapsed) : Triangle_cmd(time_elapsed);
        Vc = InverseMap(Vcmd);

        if (count < BUF_SIZE) {
            buftime[count] = time_elapsed;
            bufVcmd[count] = Vcmd;
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;
            bufomega_target[count] = omega_target;
        }

        motor_power(ON, Vc);

        if (count % 200 == 0)
            printf("  t = %6.1f s | Vcmd = %7.4f | Vc = %7.4f | omega = %8.4f\n",
                time_elapsed, Vcmd, Vc, omega);

        count++;
        WaitNextSample();

    } while (!IsEmergencyStop() && time_elapsed < T_total);

    motor_power(ON, NEUTRAL);

    if (mode == MODE_SINE)
        sprintf(filename, "%s/sine_A%.2f_F%.4f.out", outputDir, SINE_AMP, SINE_FREQ);
    else
        sprintf(filename, "%s/tri_A%.1f_T%.0f.out", outputDir, TRI_AMP, TRI_PERIOD);

    FILE* fp = fopen(filename, "w");
    if (!fp) { printf("[Error] Cannot open: %s\n", filename); return; }

    fprintf(fp, "%% K_lin=%.6f  p_cw=[%.6f %.6f %.6f]  p_ccw=[%.6f %.6f %.6f]\n",
        K_LIN, CW_C2, CW_C1, CW_C0, CCW_C2, CCW_C1, CCW_C0);
    fprintf(fp, "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
        "Time[s]", "Vcmd[V]", "Vc[V]", "Vg[V]", "Pot[V]", "Omega[rad/s]", "Omega_target[rad/s]");
    for (int i = 0; i < count; i++)
        fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
            buftime[i], bufVcmd[i], bufVc[i], bufVg[i], bufVpot[i], bufomega[i], bufomega_target[i]);

    fclose(fp);
    printf("[Saved] %s  (%d samples)\n", filename, count);
}


void RunBode(void)
{
    double t = 0.0;
    double freq = 0.0;
    double T_total = 0.0;
    double total_est = 0.0;
    int    n_cycles = 0;
    int    N_total = 0;
    char   fname[256];

    const char* outputDir = "bode_data";
    _mkdir(outputDir);

    for (int i = 0; i < N_FREQS; i++)
        total_est += (1.0 / TEST_FREQS[i]) * GetNCycles(TEST_FREQS[i]);

    printf("============================================================\n");
    printf("  [MODE 4] Frequency Response (Bode Plot)\n");
    printf("  Sine amplitude : %.2f V  (Vcmd)\n", BODE_SINE_AMP);
    printf("  Frequencies    : %d points  (0.1 ~ 15 Hz)\n", N_FREQS);
    printf("  Cycles/freq    : adaptive (f<%.1f: %d, f<%.1f: %d, else: %d)\n",
        FREQ_THR_LOW, N_CYCLES_LOW, FREQ_THR_MID, N_CYCLES_MID, N_CYCLES_HIGH);
    printf("  Estimated time : ~%.0f sec (%.1f min)\n", total_est, total_est / 60.0);
    printf("============================================================\n\n");
    printf("[Step 1] Turn on gimbal switch, then press [Enter].\n\n");
    getchar();
    GetAsyncKeyState(VK_SPACE);

    motor_power(ON, NEUTRAL);

    for (int fi = 0; fi < N_FREQS && !IsEmergencyStop(); fi++)
    {
        freq = TEST_FREQS[fi];
        n_cycles = GetNCycles(freq);
        T_total = (1.0 / freq) * n_cycles;
        N_total = (int)(T_total * SAMPLING_FREQ);
        memorySet();

        if (N_total > BODE_N_MAX) {
            printf("[WARN] f=%.1f Hz skipped (N_total=%d exceeds BODE_N_MAX=%d)\n",
                freq, N_total, BODE_N_MAX);
            continue;
        }

        printf("------------------------------------------------------------\n");
        printf("[%d/%d] f=%.1fHz | cycles=%d | T=%.1fs | N_total=%d\n",
            fi + 1, N_FREQS, freq, n_cycles, T_total, N_total);

        time_init = GetWindowTime();
        count = 0;

        do
        {
            DAQ_ReadSample();
            t = count * SAMPLING_TIME;

            Vcmd = BODE_SINE_AMP * sin(2.0 * UNIT_PI * freq * t);
            Vc = InverseMap(Vcmd);

            // ★ 버퍼 저장은 count++ 이전에
            buftime[count] = t;
            bufVcmd[count] = Vcmd;
            bufVc[count] = Vc;
            bufVg[count] = Vg;
            bufVpot[count] = Vpot;
            bufomega[count] = omega;
            bufomega_target[count] = omega_target;

            motor_power(ON, Vc);

            count++;
            WaitNextSample();

        } while (count < N_total && !IsEmergencyStop());

        if (IsEmergencyStop()) {
            printf("\n[EMERGENCY STOP] Spacebar pressed!\n");
            break;
        }

        motor_power(ON, NEUTRAL);

        sprintf(fname, "%s/raw_f%.2fHz.out", outputDir, freq);
        FILE* fp = fopen(fname, "w+t");
        if (fp) {
            fprintf(fp, "%% f=%.2fHz  n_cycles=%d  N_total=%d  dt=%.6f\n",
                freq, n_cycles, N_total, SAMPLING_TIME);
            fprintf(fp, "%% Amp=%.4fV  K_lin=%.6f\n", BODE_SINE_AMP, K_LIN);
            fprintf(fp, "%-20s %-20s %-20s %-20s %-20s %-20s %-20s\n",
                "Time[s]", "Vcmd[V]", "Vc[V]", "Vg[V]", "Pot[V]",
                "Omega[rad/s]", "Omega_target[rad/s]");
            for (int k = 0; k < count; k++)   // ★ N_total → count (비상정지 대응)
                fprintf(fp, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
                    buftime[k], bufVcmd[k], bufVc[k], bufVg[k], bufVpot[k],
                    bufomega[k], bufomega_target[k]);
            fclose(fp);
            printf("  -> Saved: %s  (%d samples)\n", fname, count);
        }
        else {
            printf("  !! File open failed: %s\n", fname);
        }

        BusyWait_ms(1000.0);
    }

    motor_power(ON, NEUTRAL);
    printf("\n[MODE 4 Done] Output folder: %s\n\n", outputDir);
}

int GetNCycles(double freq)
{
    if (freq < FREQ_THR_LOW) return N_CYCLES_LOW;
    else if (freq < FREQ_THR_MID) return N_CYCLES_MID;
    else                          return N_CYCLES_HIGH;
}
