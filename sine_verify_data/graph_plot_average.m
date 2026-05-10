% =========================================================
%  sine_verify_plot.m
%  Motor Linearization Verification - Sine Wave Analysis
%
%  1. 원본 데이터 플롯
%  2. 주기 평균 & 표준편차 계산
%  3. lsqcurvefit 으로 평균/표준편차 피팅
%  4. 피팅 결과 플롯
% =========================================================

clear; clc; close all;

% =========================================================
%  [1] 데이터 로드
% =========================================================
D = readmatrix('sine_verify_data\sine_verify_A0.45_F0.0250_3cyc.out', ...
               'FileType','text', 'NumHeaderLines', 9);

t      = D(:,1);   % Time [s]
vcmd   = D(:,2);   % Vcmd_ref [V]
om     = D(:,6);   % Measured omega [rad/s]
om_tgt = D(:,7);   % Target omega [rad/s]

% =========================================================
%  [2] 원본 데이터 플롯
% =========================================================
figure('Name', '원본 데이터');

subplot(2,1,1);
plot(t, vcmd, 'b', 'LineWidth', 1.0);
ylabel('Vcmd_{ref} [V]');
title('원본 데이터');
grid on;

subplot(2,1,2); hold on;
plot(t, om_tgt, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target \omega');
plot(t, om,     'r',   'LineWidth', 0.8, 'DisplayName', 'Measured \omega');
ylabel('\omega [rad/s]');
xlabel('Time [s]');
legend('Location', 'best');
grid on;

% =========================================================
%  [3] 주기 평균 & 표준편차
% =========================================================
T_period = 1 / 0.025;            % 한 주기 [sec] = 40 sec
Fs       = 200;                   % 샘플링 주파수 [Hz]
N_cycle  = round(T_period * Fs); % 한 주기당 샘플 수 = 8000

n_total  = length(om);
n_cycles = floor(n_total / N_cycle);  % 완전한 주기 수

fprintf('=== 주기 분석 ===\n');
fprintf('  총 샘플   : %d\n',   n_total);
fprintf('  주기 샘플 : %d\n',   N_cycle);
fprintf('  사용 주기 : %d\n\n', n_cycles);

% 완전한 주기만 사용 → [N_cycle x n_cycles] 행렬로 reshape
om_mat  = reshape(om(1 : N_cycle*n_cycles),     N_cycle, n_cycles);
tgt_mat = reshape(om_tgt(1 : N_cycle*n_cycles), N_cycle, n_cycles);

om_mean  = mean(om_mat,  2);    % 각 시점의 평균   (N_cycle x 1)
om_std   = std(om_mat,   0, 2); % 각 시점의 표준편차
tgt_mean = mean(tgt_mat, 2);

t_cycle = (0 : N_cycle-1)' / Fs;   % 0 ~ T_period 시간 축

fprintf('=== 주기 평균 통계 (raw) ===\n');
fprintf('  평균 표준편차 : %.4f rad/s\n', mean(om_std, 'omitnan'));
fprintf('  최대 표준편차 : %.4f rad/s\n', max(om_std));

% =========================================================
%  [4] lsqcurvefit 피팅
% =========================================================
options = optimoptions('lsqcurvefit', 'Display', 'off');

% ---- 평균 피팅: A*sin(2*pi*f*t + phi) + C ----
sine_model = @(p, t) p(1)*sin(2*pi*p(2)*t + p(3)) + p(4);

p0 = [4.0,  0.025,  0.0,  0.0];
lb = [0,    0.020, -pi,  -1.0];
ub = [6.0,  0.030,  pi,   1.0];

p_mean = lsqcurvefit(sine_model, p0, t_cycle, om_mean,  lb, ub, options);
p_tgt  = lsqcurvefit(sine_model, p0, t_cycle, tgt_mean, lb, ub, options);

om_mean_fit  = sine_model(p_mean, t_cycle);
tgt_mean_fit = sine_model(p_tgt,  t_cycle);

fprintf('\n=== 피팅 결과 (Measured omega) ===\n');
fprintf('  A   = %.4f rad/s\n', p_mean(1));
fprintf('  f   = %.6f Hz\n',   p_mean(2));
fprintf('  phi = %.4f rad\n',  p_mean(3));
fprintf('  C   = %.4f rad/s\n',p_mean(4));

fprintf('\n=== 피팅 결과 (Target omega) ===\n');
fprintf('  A   = %.4f rad/s\n', p_tgt(1));
fprintf('  f   = %.6f Hz\n',   p_tgt(2));
fprintf('  phi = %.4f rad\n',  p_tgt(3));
fprintf('  C   = %.4f rad/s\n',p_tgt(4));

% ---- 표준편차 피팅: |A*sin(2*pi*f*t + phi)| + C ----
%      영점 교차 부근에서 std 작고, 피크 부근에서 큰 경향 반영
std_model = @(p, t) abs(p(1)*sin(2*pi*p(2)*t + p(3))) + p(4);

p0_std = [0.3,  0.025,  0.0,  0.05];
lb_std = [0,    0.020, -pi,   0.0 ];
ub_std = [1.0,  0.030,  pi,   0.5 ];

p_std      = lsqcurvefit(std_model, p0_std, t_cycle, om_std, lb_std, ub_std, options);
om_std_fit = std_model(p_std, t_cycle);

fprintf('\n=== 표준편차 피팅 결과 ===\n');
fprintf('  mean std = %.4f rad/s\n', mean(om_std_fit));
fprintf('  max  std = %.4f rad/s\n', max(om_std_fit));

% =========================================================
%  [5] 피팅 결과 플롯
% =========================================================
figure('Name', '주기 평균 피팅 결과');
hold on;

% 표준편차 밴드 (fill)
fill([t_cycle; flipud(t_cycle)], ...
     [om_mean_fit + om_std_fit; flipud(om_mean_fit - om_std_fit)], ...
     'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none', ...
     'DisplayName', '\pm1\sigma band');

% 표준편차 상한/하한 점선
plot(t_cycle, om_mean_fit + om_std_fit, 'r--', 'LineWidth', 1.2, ...
     'DisplayName', '+1\sigma');
plot(t_cycle, om_mean_fit - om_std_fit, 'b--', 'LineWidth', 1.2, ...
     'DisplayName', '-1\sigma');

% 평균 곡선
plot(t_cycle, tgt_mean_fit, 'k--', 'LineWidth', 2.0, ...
     'DisplayName', 'Target \omega (fit)');
plot(t_cycle, om_mean_fit,  'r',   'LineWidth', 2.0, ...
     'DisplayName', 'Measured \omega (fit)');

xlabel('Time [s]');
ylabel('\omega [rad/s]');
title(sprintf('주기 평균 피팅 (N=%d cycles),  mean std = %.4f rad/s', ...
              n_cycles, mean(om_std_fit)));
legend('Location', 'best');
grid on;