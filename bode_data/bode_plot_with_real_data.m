% ---------------------------------------------%
%         transfer function estimation         %
% ---------------------------------------------%
close all; clear all; clc;

% 헤더 2줄 skip (주석행 + 컬럼명 행)
data = readmatrix("bode_result.out", 'FileType', 'text', ...
                  'NumHeaderLines', 3);

tblOmega    = 2 * pi * data(:,1);       % Hz → rad/s
tblMagAtt   = data(:,2);                % [dB]
tblPhsDelay = data(:,3) * pi / 180;    % deg → rad

tblMagLin   = 10.^(tblMagAtt / 20);
tblFreqResp = tblMagLin .* exp(1j * tblPhsDelay);

Nnum = 0;
Nden = 1;
[num, den] = invfreqs(tblFreqResp, tblOmega, Nnum, Nden);
EstTF = tf(num, den);

% ── Bode plot ──────────────────────────────────────────────
figure('Position', [100 100 800 600]);

% 전달함수 곡선
[mag, phs, wout] = bode(EstTF);
mag = squeeze(mag);          % linear
phs = squeeze(phs);          % deg
mag_dB = 20*log10(mag);

% 측정 데이터 (Hz → rad/s는 이미 tblOmega)
meas_freq_hz = data(:,1);
meas_mag_dB  = data(:,2);
meas_phs_deg = data(:,3);

% ── 크기(Magnitude) subplot ────────────────────────────────
ax1 = subplot(2,1,1);
semilogx(wout, mag_dB, 'b-', 'LineWidth', 1.8);   hold on;
semilogx(tblOmega, meas_mag_dB, 'ro', ...
    'MarkerSize', 7, 'LineWidth', 1.5, ...
    'MarkerFaceColor', 'r');

% 데이터 값 레이블
for k = 1:length(meas_freq_hz)
    text(tblOmega(k), meas_mag_dB(k) + 1.5, ...
        sprintf('%.1fdB', meas_mag_dB(k)), ...
        'FontSize', 7, 'Color', [0.8 0 0], ...
        'HorizontalAlignment', 'center');
end

grid on; ylabel('Magnitude (dB)');
title('Bode Plot — Estimated TF vs Measured Data');
legend('Estimated TF (1st order)', 'Measured', 'Location', 'southwest');
xlim([min(tblOmega)*0.8, max(tblOmega)*1.3]);

% ── 위상(Phase) subplot ────────────────────────────────────
ax2 = subplot(2,1,2);
semilogx(wout, phs, 'b-', 'LineWidth', 1.8);   hold on;
semilogx(tblOmega, meas_phs_deg, 'ro', ...
    'MarkerSize', 7, 'LineWidth', 1.5, ...
    'MarkerFaceColor', 'r');

% 데이터 값 레이블
for k = 1:length(meas_freq_hz)
    text(tblOmega(k), meas_phs_deg(k) - 6, ...
        sprintf('%.1f°', meas_phs_deg(k)), ...
        'FontSize', 7, 'Color', [0.8 0 0], ...
        'HorizontalAlignment', 'center');
end

grid on;
xlabel('Frequency (rad/s)');
ylabel('Phase (deg)');
legend('Estimated TF (1st order)', 'Measured', 'Location', 'southwest');
xlim([min(tblOmega)*0.8, max(tblOmega)*1.3]);

% 두 subplot x축 연동
linkaxes([ax1, ax2], 'x');