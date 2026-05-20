D = readmatrix('sine_verify_data\Sine_A1.0000_T40.0000_1cyc.out', ...
               'FileType','text', 'NumHeaderLines', 12);
t      = D(:,1);  vcmd = D(:,2);
om_tgt = D(:,7);  om   = D(:,6);

figure;
subplot(2,1,1);
plot(t, vcmd, 'b'); ylabel('Vcmd_{ref} [V]'); grid on;

subplot(2,1,2); hold on;
plot(t, om_tgt, 'k--', 'DisplayName', 'Target \omega');
plot(t, om,     'r',   'DisplayName', 'Measured \omega');
ylabel('\omega [rad/s]'); xlabel('Time [s]');
legend; grid on;