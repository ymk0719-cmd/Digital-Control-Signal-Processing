data = readmatrix('bode_data\bode_result.out', ...
               'FileType','text', 'NumHeaderLines', 6);
freq      = data(:,1);
gain_dB   = data(:,2);
phase_deg = data(:,3);

figure;
subplot(2,1,1);
semilogx(freq, gain_dB, 'bo-', 'LineWidth', 1.5, 'MarkerSize', 8);
ylabel('Gain (dB)'); grid on; title('Bode Plot');

subplot(2,1,2);
semilogx(freq, phase_deg, 'ro-', 'LineWidth', 1.5, 'MarkerSize', 8);
ylabel('Phase (deg)'); xlabel('Frequency (Hz)'); grid on;