%% ============================================================
%  V_cmd vs Omega - Motor Sweep (180 steps)
%  - Omega: 마지막 2초 평균
%  - CW / CCW 구분하여 단일 그래프에 플롯
%% ============================================================
clear; clc; close all;

%% --- 파일 경로 설정 ---
data_dir = 'C:\Users\ADMIN\source\repos\Digital-Control-Signal-Processing\motor_sweep_data';
files = dir(fullfile(data_dir, 'step_*.out'));
files = sort({files.name});

% 3자리 번호 파일만 사용 (새 실험: step_001, step_002, ...)
files = files(~cellfun(@isempty, regexp(files, 'step_\d{3}_')));
fprintf('총 파일 수: %d\n', numel(files));

%% --- 데이터 추출 ---
Vcmd_CW  = [];  Omega_CW  = [];
Vcmd_CCW = [];  Omega_CCW = [];

for i = 1:numel(files)
    fname = files{i};

    % 파일명 파싱: step_161_V4.55_CW.out  ← 소수점(.) 패턴으로 수정
    tok = regexp(fname, 'step_\d+_V(\d+)\.(\d+)_(CW|CCW)\.out', 'tokens');

    if isempty(tok)
        fprintf('Skip (unrecognized): %s\n', fname);
        continue;
    end

    vcmd      = str2double(tok{1}{1}) + str2double(tok{1}{2}) * 0.01;
    direction = tok{1}{3};

    % 파일 읽기 (헤더 8줄 스킵)
    fid = fopen(fullfile(data_dir, fname), 'r');
    raw = textscan(fid, '%f %f %f %f %f', 'HeaderLines', 8, 'CommentStyle', '%');
    fclose(fid);

    t     = raw{1};
    omega = raw{5};

    if isempty(t)
        fprintf('No data: %s\n', fname);
        continue;
    end

    % 마지막 2초 평균
    t_end      = t(end);
    mask       = t >= (t_end - 2.0);
    omega_mean = mean(omega(mask));

    if strcmp(direction, 'CW')
        Vcmd_CW(end+1)  = vcmd;
        Omega_CW(end+1) = omega_mean;
    else
        Vcmd_CCW(end+1)  = vcmd;
        Omega_CCW(end+1) = omega_mean;
    end
end

% Vcmd 기준 정렬
[Vcmd_CW,  idx] = sort(Vcmd_CW);   Omega_CW  = Omega_CW(idx);
[Vcmd_CCW, idx] = sort(Vcmd_CCW);  Omega_CCW = Omega_CCW(idx);

fprintf('CW: %d개, CCW: %d개\n', numel(Vcmd_CW), numel(Vcmd_CCW));

%% --- 플롯 ---
figure(1); clf;
hold on; grid on;

scatter(Vcmd_CW,  Omega_CW,  60, 'b', 'filled', 'DisplayName', 'CW');
scatter(Vcmd_CCW, Omega_CCW, 60, 'r', 'filled', 'DisplayName', 'CCW');
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');

xlabel('V_{cmd} [V]',    'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('V_{cmd} vs \omega', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gcf, 'Position', [100, 100, 900, 550]);