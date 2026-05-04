%% ============================================================
%  V_cmd vs Omega - Load from .out files
%  - 앞 5초 버리고 뒤 5초 Omega 평균 계산
%  - CW / CCW 구분하여 단일 그래프에 플롯
%% ============================================================

clear; clc; close all;

%% --- 파일 경로 설정 ---
data_dir = 'C:\Users\ADMIN\source\repos\Digital-Control-Signal-Processing\motor_sweep_data';

files = dir(fullfile(data_dir, 'step_*.out'));
files = sort({files.name});

%% --- 데이터 추출 ---
Vcmd_CW  = [];  Omega_CW  = [];
Vcmd_CCW = [];  Omega_CCW = [];

for i = 1:numel(files)
    fname = files{i};

    % 파일명 파싱: 소수점(V0.3) 또는 언더스코어(V0_3) 형식 둘 다 지원
    % 형식 1: step_01_V2_6_CW.out  (언더스코어)
    tok = regexp(fname, 'step_\d+_V(\d+)_(\d+)_(CW|CCW)\.out', 'tokens');
    if ~isempty(tok)
        vcmd      = str2double(tok{1}{1}) + str2double(tok{1}{2}) * 0.1;
        direction = tok{1}{3};
    else
        % 형식 2: step_01_V2.6_CW.out  (소수점)
        tok = regexp(fname, 'step_\d+_V([\d.]+)_(CW|CCW)\.out', 'tokens');
        if ~isempty(tok)
            vcmd      = str2double(tok{1}{1});
            direction = tok{1}{2};
        else
            fprintf('Skip (unrecognized): %s\n', fname);
            continue;
        end
    end

    % 파일 읽기 (헤더 7줄 스킵)
    fid = fopen(fullfile(data_dir, fname), 'r');
    raw = textscan(fid, '%f %f %f %f %f', 'HeaderLines', 7, 'CommentStyle', '%');
    fclose(fid);

    t     = raw{1};
    omega = raw{5};

    if isempty(t)
        fprintf('No data: %s\n', fname);
        continue;
    end

    % 뒤 5초 구간 마스크
    t_end   = t(end);
    t_start = t_end - 5.0;
    mask    = t >= t_start;

    omega_mean = mean(omega(mask));

    if strcmp(direction, 'CW')
        Vcmd_CW(end+1)  = vcmd;
        Omega_CW(end+1) = omega_mean;
    else
        Vcmd_CCW(end+1)  = vcmd;
        Omega_CCW(end+1) = omega_mean;
    end

    fprintf('%s -> Vcmd=%.1f V, Dir=%s, Omega_mean=%.4f rad/s\n', ...
            fname, vcmd, direction, omega_mean);
end

% Vcmd 기준 정렬
[Vcmd_CW,  idx] = sort(Vcmd_CW);   Omega_CW  = Omega_CW(idx);
[Vcmd_CCW, idx] = sort(Vcmd_CCW);  Omega_CCW = Omega_CCW(idx);

fprintf('\n총 CW: %d개, CCW: %d개\n', numel(Vcmd_CW), numel(Vcmd_CCW));

%% --- 플롯 ---
figure(1);
hold on; grid on;

scatter(Vcmd_CW,  Omega_CW,  60, 'b', 'filled', 'DisplayName', 'CW');
scatter(Vcmd_CCW, Omega_CCW, 60, 'r', 'filled', 'DisplayName', 'CCW');

yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');

xlabel('V_{cmd} [V]',   'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('V_{cmd} vs \omega', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gcf, 'Position', [100, 100, 900, 550]);