% =========================================================
%  plot_wave_verify.m  (수정본)
%
%  문제 수정:
%    1. NumHeaderLines = 2  (주석행 + 컬럼명행)
%    2. 파일 내 깨진 행(컬럼 병합 오염) 자동 제거
%    3. Vcmd 컬럼은 [rad/s] 단위임을 반영 (Triangle_cmd 출력)
%
%  컬럼 순서:
%    1:Time[s]  2:Vcmd(=omega_ref)[rad/s]  3:Vc[V]
%    4:Vg[V]    5:Pot[V]   6:Omega[rad/s]  7:Omega_target[rad/s]
% =========================================================

clear; clc; close all;

% ── 파일 경로 ────────────────────────────────────────────
tri_file  = 'triangle_verify_data\tri_A1.0_T40.out';
sine_file = 'sine_verify_data\sine_A5.00_F1.0000.out';

% ── 플랏 공통 설정 ────────────────────────────────────────
LW = 1.5;
FS = 11;

% ==========================================================
%  1. 삼각파 (Triangle Wave)
% ==========================================================
if isfile(tri_file)
    D = safe_load(tri_file);
    plot_wave(D, '삼각파 (Triangle Wave)', LW, FS);
else
    fprintf('[경고] 파일 없음: %s\n', tri_file);
end

% ==========================================================
%  2. 정현파 (Sine Wave)
% ==========================================================
if isfile(sine_file)
    D = safe_load(sine_file);
    plot_wave(D, '정현파 (Sine Wave)', LW, FS);
else
    fprintf('[경고] 파일 없음: %s\n', sine_file);
end


% ==========================================================
%  공통 플랏 함수
% ==========================================================
function plot_wave(D, title_str, LW, FS)
    t        = D(:,1);
    Vc       = D(:,3);
    Vg       = D(:,4);
    Pot      = D(:,5);
    omega    = D(:,6);
    om_tgt   = D(:,7);   % = K_lin * Vcmd

    % ── Figure 1: 메인 3단 ────────────────────────────────
    figure('Name', title_str, 'NumberTitle','off', ...
           'Position', [100 80 980 740]);
    sgtitle(title_str, 'FontSize', 14, 'FontWeight','bold');

    % (1) 모터 인가 전압 Vc
    ax1 = subplot(3,1,1);
    plot(t, Vc, 'b-', 'LineWidth', LW);
    ylabel('V_c [V]', 'FontSize', FS);
    title('Motor Voltage (Vc)');
    yline(2.5, 'k--', 'Neutral', 'FontSize', FS-1);
    grid on; xlim([t(1) t(end)]);

    % (2) 각속도 추종
    ax2 = subplot(3,1,2);
    plot(t, om_tgt, 'b-',  'LineWidth', LW, 'DisplayName', '\omega_{target}'); hold on;
    plot(t, omega,  'r--', 'LineWidth', LW, 'DisplayName', '\omega (meas)');
    ylabel('\omega [rad/s]', 'FontSize', FS);
    title('Angular Velocity Tracking');
    legend('Location','best', 'FontSize', FS-1);
    grid on; xlim([t(1) t(end)]);

    % (3) 원시 센서
    ax3 = subplot(3,1,3);
    yyaxis left;
    plot(t, Vg, 'g-', 'LineWidth', LW);
    ylabel('V_g – Gyro [V]', 'FontSize', FS);
    yyaxis right;
    plot(t, Pot, 'm-', 'LineWidth', LW);
    ylabel('V_{pot} [V]', 'FontSize', FS);
    xlabel('Time [s]', 'FontSize', FS);
    title('Raw Sensor Signals');
    grid on; xlim([t(1) t(end)]);

    linkaxes([ax1 ax2 ax3], 'x');
    set([ax1 ax2 ax3], 'FontSize', FS);

    % ── Figure 2: 추종 오차 ───────────────────────────────
    figure('Name', [title_str ' – Error'], 'NumberTitle','off', ...
           'Position', [150 150 980 300]);
    err = omega - om_tgt;
    plot(t, err, 'k-', 'LineWidth', LW); hold on;
    yline(0, '--', 'Color', [0.5 0.5 0.5]);
    xlabel('Time [s]', 'FontSize', FS);
    ylabel('\omega error [rad/s]', 'FontSize', FS);
    title(sprintf('%s  Tracking Error  |  RMS = %.4f rad/s', ...
          title_str, rms(err)), 'FontSize', FS);
    grid on; xlim([t(1) t(end)]);
    set(gca, 'FontSize', FS);
end


% ==========================================================
%  safe_load: 헤더 2줄 스킵 + 깨진 행 자동 제거
%
%  헤더 구조:
%    Line 1: % K_lin=...          <- '%' 시작 주석
%    Line 2: Time[s] Vcmd[V] ...  <- 알파벳 컬럼명
%    Line 3~: 숫자 데이터 (7컬럼)
% ==========================================================
function M = safe_load(filepath)
    fid = fopen(filepath, 'r');
    if fid < 0
        error('파일을 열 수 없습니다: %s', filepath);
    end

    rows   = {};
    n_skip = 0;

    while ~feof(fid)
        raw = fgetl(fid);
        if ~ischar(raw), break; end

        line = strtrim(raw);
        if isempty(line), continue; end

        % 헤더 스킵: '%' 주석행 또는 알파벳으로 시작하는 컬럼명행
        if line(1) == '%' || isletter(line(1)), continue; end

        % 공백 분리 후 숫자 변환
        vals = str2double(strsplit(line));

        % 정상 행 조건: 정확히 7컬럼 + 모두 유한한 수
        if numel(vals) == 7 && all(isfinite(vals))
            rows{end+1} = vals; %#ok<AGROW>
        else
            n_skip = n_skip + 1;
        end
    end
    fclose(fid);

    if isempty(rows)
        error('유효한 데이터 행이 없습니다: %s', filepath);
    end

    M = vertcat(rows{:});
    fprintf('[로드 완료] %s\n  유효 %d행 / 제거(오염) %d행\n\n', ...
            filepath, size(M,1), n_skip);
end