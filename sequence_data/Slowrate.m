%% Slew Rate Measurement - Segment Plot
% 파일을 읽고, Vcmd가 바뀌는 지점으로 세그먼트를 나눠서 각각 plot
clear; clc; close all;
%% 1. 데이터 로드 (헤더 11줄 스킵)
filename = 'continuous_sequence_seg3s_20seg.out';
data = readmatrix(filename, 'NumHeaderLines', 12, 'FileType', 'text');
time         = data(:, 1);  % [s]
vcmd_ref     = data(:, 2);  % [V]
omega        = data(:, 6);  % [rad/s] 실측값
omega_target = data(:, 7);  % [rad/s] 목표값
%% 2. Vcmd가 바뀌는 인덱스 자동 탐지
vcmd_diff  = diff(vcmd_ref);
trans_idx  = find(vcmd_diff ~= 0);   % 전환 직전 인덱스
trans_time = time(trans_idx + 1);    % 전환이 일어난 실제 시간
% 세그먼트 경계 인덱스 (시작=1, 각 전환점, 끝=마지막)
seg_starts = [1;        trans_idx + 1];
seg_ends   = [trans_idx; length(time)];
N_seg      = length(seg_starts);
fprintf('총 %d개 세그먼트 감지됨\n', N_seg);
fprintf('%4s  %10s  %10s  %12s  %12s\n', 'Seg', 'T_start[s]', 'T_end[s]', 'Vcmd[V]', 'Omega_tgt');
for k = 1:N_seg
    fprintf('%4d  %10.4f  %10.4f  %12.4f  %12.4f\n', ...
        k, time(seg_starts(k)), time(seg_ends(k)), ...
        vcmd_ref(seg_starts(k)), omega_target(seg_starts(k)));
end
%% 3. 전체 개요 Plot (Time vs Omega & Omega_target)
figure('Name', 'Overall', 'Position', [100 100 1200 400]);
plot(time, omega,        'b', 'LineWidth', 1.0, 'DisplayName', '\omega (measured)');
hold on;
plot(time, omega_target, 'r--', 'LineWidth', 1.2, 'DisplayName', '\omega_{target}');
% 세그먼트 경계 수직선
for k = 2:N_seg
    xline(time(seg_starts(k)), 'k:', 'Alpha', 0.4);
end
xlabel('Time [s]'); ylabel('\omega [rad/s]');
title('Full Sequence: \omega vs \omega_{target}');
legend('Location', 'best');
grid on;
%% 4. 모든 Step 세그먼트 오버레이 Plot (0~3초 기준으로 겹쳐서)
step_segs = find(vcmd_ref(seg_starts) ~= 0);

figure('Name', 'Step Overlay', 'Position', [100 550 800 500]);
hold on;

colors = lines(length(step_segs));

for i = 1:length(step_segs)
    k   = step_segs(i);
    idx = seg_starts(k) : seg_ends(k);
    t_k = time(idx) - time(seg_starts(k));

    mask = t_k <= 3.0;

    plot(t_k(mask), omega(idx(mask)), ...
        'Color', colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('Seg%d Vcmd=%.2fV', k, vcmd_ref(seg_starts(k))));
end

xlabel('t [s]');
ylabel('\omega [rad/s]');
title('All Step Segments Overlaid (0–3 s window)');
legend('Location', 'best', 'FontSize', 8);
grid on;
xlim([0 3]);
hold off;
%% 5. Slew Rate 추정 (각 step 세그먼트의 최대 dω/dt)
fprintf('\n=== 세그먼트별 Slew Rate 추정 ===\n');
fprintf('%4s  %8s  %14s  %14s\n', 'Seg', 'Vcmd[V]', 'Omega_tgt', 'Max dω/dt [rad/s²]');
for i = 1:length(step_segs)
    k   = step_segs(i);
    idx = seg_starts(k) : seg_ends(k);
    d_omega = diff(omega(idx)) ./ diff(time(idx));
    [max_sr, ~] = max(abs(d_omega));
    fprintf('%4d  %8.2f  %14.4f  %14.2f\n', ...
        k, vcmd_ref(seg_starts(k)), omega_target(seg_starts(k)), max_sr);
end