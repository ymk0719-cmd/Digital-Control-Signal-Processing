clear all; close all; clc;
Ts = 0.005;
Tf = 40.0;
time = 0:Ts:Tf;
num2 = 8480;
den2 = [1 52.16 1266];
num1 =  129.3;
den1 = [1 16.19];
f_in = 0.025;
A = 1.0;
period = 1 / f_in;

% 삼각파 생성
phase = mod(time, period) / period;
u = zeros(size(time));
u(phase < 0.25)                  = A * (4.0 * phase(phase < 0.25));
u(phase >= 0.25 & phase < 0.75)  = A * (2.0 - 4.0 * phase(phase >= 0.25 & phase < 0.75));
u(phase >= 0.75)                 = A * (4.0 * phase(phase >= 0.75) - 4.0);

% Transfer function
Gs = tf(num1, den1); % 1차로 추정한 전달함수 
Gs2 = tf(num2, den2) ; % 2차로 추정한 전달함수 
response = lsim(Gs2, u, time);

% Step response: step(sys, t) 형태로 사용
[step1, t_step] = step(Gs, time);
[step2, t_step] = step(Gs2, time);

% 실측 데이터 로드 (헤더 7줄 스킵)
data = readmatrix('triangle_verify_cycle_avg.out', 'FileType', 'text');
t_meas        = data(8:end, 1);
omega_target  = data(8:end, 3);
omega_meas    = data(8:end, 4);

% Figure 1: 삼각파 시뮬레이션 vs 실측
figure(1); hold on;
plot(time,   u,            '--',  'LineWidth', 1.5, 'DisplayName', 'Input (Triangle)');
plot(time,   response,     '-',   'LineWidth', 2,   'DisplayName', 'Simulated Response');
plot(t_meas, omega_target, '-.',  'LineWidth', 1.5, 'DisplayName', 'Omega Target (Measured)');
plot(t_meas, omega_meas,   '-',   'LineWidth', 2,   'DisplayName', 'Omega Actual (Measured)');
xlabel('time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Amplitude [-]', 'FontSize', 12, 'FontWeight', 'bold');
legend('FontSize', 11, 'Location', 'best');
title('Triangle Wave: Simulation vs Measured', 'FontSize', 13, 'FontWeight', 'bold');
grid on; box on;

% Figure 2: Step response ✅
figure(2); hold on;
plot(t_step, step1, '-', 'LineWidth', 2, 'DisplayName', 'Step Response');
plot(t_step, step2, '-', 'LineWidth', 2, 'DisplayName', 'Step Response');
xlabel('time [sec]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Amplitude [-]', 'FontSize', 12, 'FontWeight', 'bold');
title('Step Response of G_m', 'FontSize', 13, 'FontWeight', 'bold');
legend('1st Gm', '2nd Gm', 'FontSize', 11, 'Location', 'best');
[~, idx] = min(abs(t_step - 1.5)) ;
xlim([t_step(1), t_step(idx)]);
grid on; box on;