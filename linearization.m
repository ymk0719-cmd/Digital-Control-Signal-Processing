clear; clc; close all;

%% ================================================================
%  0. 파일에서 데이터 추출 → txt 4개 저장 (기존 코드)
%% ================================================================
data_dir = 'C:\Users\ADMIN\source\repos\Digital-Control-Signal-Processing\motor_sweep_data';
files = dir(fullfile(data_dir, 'step_*.out'));
files = sort({files.name});
files = files(~cellfun(@isempty, regexp(files, 'step_\d{3}_')));
fprintf('총 파일 수: %d\n', numel(files));

Vcmd_CW  = [];  Omega_CW  = [];
Vcmd_CCW = [];  Omega_CCW = [];

for i = 1:numel(files)
    fname = files{i};
    tok = regexp(fname, 'step_\d+_V(\d+)\.(\d+)_(CW|CCW)\.out', 'tokens');
    if isempty(tok)
        fprintf('Skip (unrecognized): %s\n', fname);
        continue;
    end
    vcmd      = str2double(tok{1}{1}) + str2double(tok{1}{2}) * 0.01;
    direction = tok{1}{3};

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

% txt 4개 저장
writematrix(Vcmd_CW',   'Vc_CW.txt');
writematrix(Omega_CW',  'omega_CW.txt');
writematrix(Vcmd_CCW',  'Vc_CCW.txt');
writematrix(Omega_CCW', 'omega_CCW.txt');
fprintf('txt 4개 저장 완료\n\n');

%% ================================================================
%  1. txt 파일 로드
%% ================================================================
Vc_CW   = readmatrix('Vc_CW.txt');
om_CW   = readmatrix('omega_CW.txt');
Vc_CCW  = readmatrix('Vc_CCW.txt');
om_CCW  = readmatrix('omega_CCW.txt');

fprintf('CW  데이터: %d개,  omega 범위: [%.2f, %.2f]\n', ...
    length(om_CW),  min(om_CW),  max(om_CW));
fprintf('CCW 데이터: %d개,  omega 범위: [%.2f, %.2f]\n', ...
    length(om_CCW), min(om_CCW), max(om_CCW));

%% ================================================================
%  2. Figure 1: Vc vs omega (모터 특성 곡선)
%% ================================================================
figure(1); clf; hold on; grid on;
scatter(Vc_CW,  om_CW,  60, 'b', 'filled', 'DisplayName', 'CW');
scatter(Vc_CCW, om_CCW, 60, 'r', 'filled', 'DisplayName', 'CCW');
yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel('V_c [V]',        'FontSize', 13);
ylabel('\omega [rad/s]', 'FontSize', 13);
title('V_c vs \omega  (모터 특성 곡선)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  3. 유효 데이터 추출 (데드존 제외)
%% ================================================================
valid_CW  = om_CW  >  0.5;
valid_CCW = om_CCW < -0.5;

vc_cw_fit  = Vc_CW(valid_CW);   om_cw_fit  = om_CW(valid_CW);
vc_ccw_fit = Vc_CCW(valid_CCW);  om_ccw_fit = om_CCW(valid_CCW);

fprintf('\nCW  유효 데이터: %d개\n', sum(valid_CW));
fprintf('CCW 유효 데이터: %d개\n', sum(valid_CCW));

%% ================================================================
%  4. 2차 다항식 피팅
%% ================================================================
valid_CW  = om_CW  >  0.5;
valid_CCW = om_CCW < -0.5;

vc_cw_fit  = Vc_CW(valid_CW);   om_cw_fit  = om_CW(valid_CW);
vc_ccw_fit = Vc_CCW(valid_CCW);  om_ccw_fit = om_CCW(valid_CCW);

fprintf('CW  유효(데드존 제외): %d개,  Vc 범위: [%.2f, %.2f]\n', ...
    sum(valid_CW),  min(vc_cw_fit),  max(vc_cw_fit));
fprintf('CCW 유효(데드존 제외): %d개,  Vc 범위: [%.2f, %.2f]\n', ...
    sum(valid_CCW), min(vc_ccw_fit), max(vc_ccw_fit));

p_cw  = polyfit(vc_cw_fit,  om_cw_fit,  2);
p_ccw = polyfit(vc_ccw_fit, om_ccw_fit, 2);

fprintf('\n[CW  피팅] %.4f*Vc^2 + %.4f*Vc + %.4f\n', p_cw(1),  p_cw(2),  p_cw(3));
fprintf('[CCW 피팅] %.4f*Vc^2 + %.4f*Vc + %.4f\n', p_ccw(1), p_ccw(2), p_ccw(3));

% 피팅 곡선: 실제 유효 데이터 범위만 표시
Vc_cw_line  = linspace(min(vc_cw_fit),  max(vc_cw_fit),  200);
Vc_ccw_line = linspace(min(vc_ccw_fit), max(vc_ccw_fit), 200);

figure(1);
plot(Vc_cw_line,  polyval(p_cw,  Vc_cw_line),  'm-', 'LineWidth', 2, 'DisplayName', 'CW 피팅');
plot(Vc_ccw_line, polyval(p_ccw, Vc_ccw_line), 'g-', 'LineWidth', 2, 'DisplayName', 'CCW 피팅');
legend('Location', 'northwest', 'FontSize', 11);
xlim([0 5]); ylim([-30 30]);

%% ================================================================
%  5. K 결정
%% ================================================================
Vc_sat_CW  = 4.5;
Vc_sat_CCW = 5.0 - Vc_sat_CW;  % = 0.5V

omega_max =  polyval(p_cw,  Vc_sat_CW);
omega_min =  polyval(p_ccw, Vc_sat_CCW);
omega_sat = min(abs(omega_max), abs(omega_min));

Vcmd_max = 2.5;
K = omega_sat / Vcmd_max;

fprintf('\n[K 결정]\n');
fprintf('  Vc_sat_CW  = %.2f V  →  omega_max = %.2f rad/s\n', Vc_sat_CW,  omega_max);
fprintf('  Vc_sat_CCW = %.2f V  →  omega_min = %.2f rad/s\n', Vc_sat_CCW, omega_min);
fprintf('  omega_sat  = %.2f rad/s\n', omega_sat);
fprintf('  K          = %.4f (rad/s)/V\n\n', K);

%% ================================================================
%  6. Inverse Mapping: Vcmd → omega_target → Vc
%  ★ 변경: -2.5:0.001:2.5  (스텝 0.01 → 0.001)
%% ================================================================
Vcmd_ref     = -2.5:0.001:2.5;          % ← 변경
omega_target = K * Vcmd_ref;
Vc_mapped    = zeros(size(Vcmd_ref));

for i = 1:length(Vcmd_ref)
    om = omega_target(i);

    if om > 0.5                                      % CW
        a = p_cw(1); b = p_cw(2);
        disc = b^2 - 4*a*(p_cw(3) - om);
        if disc >= 0
            root1 = (-b + sqrt(disc)) / (2*a);
            root2 = (-b - sqrt(disc)) / (2*a);
            valid1 = (root1 >= 2.5) && (root1 <= 5.0);
            valid2 = (root2 >= 2.5) && (root2 <= 5.0);
            if valid1 && valid2
                Vc_mapped(i) = min(root1, root2);
            elseif valid1
                Vc_mapped(i) = root1;
            elseif valid2
                Vc_mapped(i) = root2;
            else
                Vc_mapped(i) = 2.5;
            end
        else
            Vc_mapped(i) = 2.5;
        end

    elseif om < -0.5                                 % CCW
        a = p_ccw(1); b = p_ccw(2);
        disc = b^2 - 4*a*(p_ccw(3) - om);
        if disc >= 0
            root1 = (-b + sqrt(disc)) / (2*a);
            root2 = (-b - sqrt(disc)) / (2*a);
            valid1 = (root1 >= 0) && (root1 <= 2.5);
            valid2 = (root2 >= 0) && (root2 <= 2.5);
            if valid1 && valid2
                Vc_mapped(i) = min(root1, root2);
            elseif valid1
                Vc_mapped(i) = root1;
            elseif valid2
                Vc_mapped(i) = root2;
            else
                Vc_mapped(i) = 2.5;
            end
        else
            Vc_mapped(i) = 2.5;
        end

    else                                             % 데드존
        Vc_mapped(i) = 2.5;
    end
end

Vc_mapped = max(0, min(5, Vc_mapped));

%% ================================================================
%  7. 선형화 검증용 omega 계산
%% ================================================================
omega_final = zeros(size(Vcmd_ref));
for i = 1:length(Vcmd_ref)
    if Vc_mapped(i) > 2.5 + 0.01
        omega_final(i) = polyval(p_cw,  Vc_mapped(i));
    elseif Vc_mapped(i) < 2.5 - 0.01
        omega_final(i) = polyval(p_ccw, Vc_mapped(i));
    else
        omega_final(i) = 0;
    end
end

%% ================================================================
%  8. Figure 2: 선형화 검증
%  ★ 변경: xlim / xline을 새 범위에 맞게 수정
%% ================================================================
figure(2); clf; hold on; grid on;
plot(Vcmd_ref, K*Vcmd_ref, 'k--', ...
    'LineWidth', 1.5, 'DisplayName', '목표 직선  \omega = K \cdot V_{cmd}');
plot(Vcmd_ref, omega_final, 'r-', ...
    'LineWidth', 2,   'DisplayName', '선형화 후 실제 출력');
yline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');  % ← 변경 (2.5 → 0)
xlabel('V_{cmd,ref} [V]', 'FontSize', 13);
ylabel('\omega [rad/s]',   'FontSize', 13);
title('선형화 검증: V_{cmd} vs \omega', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-2.5 2.5]); ylim([-30 30]);          % ← 변경
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  9. Figure 3: Lookup Table 시각화
%  ★ 변경: xlim을 새 범위에 맞게 수정
%% ================================================================
figure(3); clf; hold on; grid on;
plot(Vcmd_ref, Vc_mapped, 'b-', 'LineWidth', 2, 'DisplayName', 'Lookup Table');
yline(2.5,        'r--', 'LineWidth', 1, 'DisplayName', '중립 2.5V');
xline(0,          'r--', 'LineWidth', 1, 'HandleVisibility', 'off');  % ← 변경 (2.5 → 0)
yline(Vc_sat_CW,  'g--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Vc\\_sat CW = %.1fV', Vc_sat_CW));
yline(Vc_sat_CCW, 'g--', 'LineWidth', 1, ...
    'DisplayName', sprintf('Vc\\_sat CCW = %.1fV', Vc_sat_CCW));
xlabel('V_{cmd,ref} [V]',  'FontSize', 13);
ylabel('V_c 실제 출력 [V]', 'FontSize', 13);
title('역함수 매핑 테이블 (Lookup Table)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 11);
xlim([-2.5 2.5]); ylim([0 5]);             % ← 변경
set(gcf, 'Position', [100, 100, 900, 550]);

%% ================================================================
%  10. Lookup Table 저장
%% ================================================================
LUT = [Vcmd_ref', Vc_mapped'];
writematrix(LUT, 'lookup_table.txt', 'Delimiter', 'tab');
fprintf('[저장 완료] lookup_table.txt  (%d rows)\n', length(Vcmd_ref));
fprintf('============================================================\n');
fprintf('  K         = %.4f (rad/s)/V\n', K);
fprintf('  omega_sat = %.2f rad/s\n', omega_sat);
fprintf('  Vc_sat    = %.2f V (CW) / %.2f V (CCW)\n', Vc_sat_CW, Vc_sat_CCW);
fprintf('  Vcmd_max  = %.2f V\n', Vcmd_max);
fprintf('============================================================\n');