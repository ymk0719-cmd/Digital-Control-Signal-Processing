% Load data
Vc_CCW = load('Vc_CCW.txt');
Vc_CW  = load('Vc_CW.txt');
omega_CCW = load('omega_CCW.txt');
omega_CW  = load('omega_CW.txt');

% Scatter plot
figure;
hold on;

scatter(Vc_CCW, omega_CCW, 40, 'b', 'filled', 'DisplayName', 'CCW');
scatter(Vc_CW,  omega_CW,  40, 'r', 'filled', 'DisplayName', 'CW');

xlabel('V_c (V)', 'FontSize', 13);
ylabel('\omega (rad/s)', 'FontSize', 13);
title('V_c vs \omega', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off;