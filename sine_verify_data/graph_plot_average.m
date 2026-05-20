D = readmatrix('sine_verify_data\sine_verify_A0.45_F0.0250_3cyc.out', ...
               'FileType','text', 'NumHeaderLines', 12);
t      = D(:,1);
vcmd   = D(:,2);
om_tgt = D(:,7);
om     = D(:,6);

%% 주기 평균
Fs      = 200;
T       = 40;
N_per   = T * Fs;          % 한 주기당 샘플 수 = 8000
N_cyc   = floor(length(t) / N_per);   % 실제 완성된 주기 수

% 완성된 주기만 사용
n_use   = N_per * N_cyc;
vcmd_m  = reshape(vcmd(1:n_use),   N_per, N_cyc);
om_tgt_m= reshape(om_tgt(1:n_use), N_per, N_cyc);
om_m    = reshape(om(1:n_use),     N_per, N_cyc);

vcmd_avg   = mean(vcmd_m,   2);
om_tgt_avg = mean(om_tgt_m, 2);
om_avg     = mean(om_m,     2);

t_one = (0 : N_per-1).' / Fs;   % 0 ~ 39.995 sec

%% 플롯
figure;
subplot(2,1,1);
plot(t_one, vcmd_avg, 'b', 'LineWidth', 1.2);
ylabel('Vcmd_{ref} [V]');
title(sprintf('주기 평균', N_cyc));
grid on;

subplot(2,1,2); hold on;
plot(t_one, om_tgt_avg, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Target \omega (avg)');
plot(t_one, om_avg,     'r',   'LineWidth', 1.2, 'DisplayName', 'Measured \omega (avg)');
ylabel('\omega [rad/s]'); xlabel('Time [s]');
legend; grid on;

%% 결과 텍스트 파일 저장
out_file = 'triangle_verify_cycle_avg.out';   % 현재 폴더에 바로 저장

fid = fopen(out_file, 'w');
if fid == -1
    error('파일 열기 실패: %s', out_file);
end

fprintf(fid, '%% Motor Triangle Wave Linearization Verification - Cycle Average\n');
fprintf(fid, '%% Period      = %.1f [sec]\n', T);
fprintf(fid, '%% Cycles used = %d\n', N_cyc);
fprintf(fid, '%% Fs          = %d [Hz]\n', Fs);
fprintf(fid, '%% N per cycle = %d\n\n', N_per);
fprintf(fid, '%-20s %-20s %-20s %-20s\n', ...
    'Time[s]', 'Vcmd_ref_avg[V]', 'Omega_target_avg[rad/s]', 'Omega_avg[rad/s]');

for i = 1 : N_per
    fprintf(fid, '%-20.10f %-20.10f %-20.10f %-20.10f\n', ...
        t_one(i), vcmd_avg(i), om_tgt_avg(i), om_avg(i));
end

fclose(fid);
fprintf('저장 완료: %s  (%d rows)\n', out_file, N_per);