D = readmatrix('sine_verify_data/sine_verify_A1.5_T10_5cyc.out', ...
               'FileType','text', 'NumHeaderLines', 12);
t      = D(:,1);  vcmd = D(:,2);
om_tgt = D(:,7);  om   = D(:,6);

figure;
subplot(2,1,1);
plot(t, vcmd, 'b'); ylabel('Vcmd_{ref} [V]'); grid on;
title('Sine Wave Linearization Verification');

subplot(2,1,2); hold on;
plot(t, om_tgt, 'k--', 'DisplayName', 'Target \omega');
plot(t, om,     'r',   'DisplayName', 'Measured \omega');
ylabel('\omega [rad/s]'); xlabel('Time [s]');
legend; grid on;