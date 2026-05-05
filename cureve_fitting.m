% --------------------------------------------------
%         2nd-order curveFitting of Vo-Vc (CW only)
% --------------------------------------------------
clear all; close all; clc;

data_omega = readmatrix("omega_CW.txt") ;
data_Vc = readmatrix("Vc_CW.txt") ;

% extract valid data only
valid_omega = data_omega(3:end) ;
valid_Vc = data_Vc(3:end) ;

% curve fitting
p = polyfit(valid_Vc, valid_omega, 2) ;
omega_fitted = polyval(p, data_Vc) ;

% 1. set Vsat (physical max value) 
Vc_sat = 4; 
omega_max = polyval(p, Vc_sat); % 2차식 기반 최대 속도

% 2. K값 결정 (Vcmd 5V일 때 최대 속도가 나오도록)
K = omega_max / Vc_sat; 

% 3. Mapping Function (Inverse Mapping)
Vcmd = 0: 0.1: 5 ;
omega_target = K * Vcmd ;
% p(1)*Vc^2 + p(2)*Vc + (p(3) - omega_target) = 0 수식을 풀어 Vc 도출

a = p(1); b = p(2); c = p(3);
Vc_mapped = (-b + sqrt(b^2 - 4 * a * (c - omega_target))) / (2 * a);

omega_final = polyval(p, Vc_mapped) ;

figure; % final linearized line
plot(Vcmd, omega_final, 'r--', 'LineWidth', 2);
title('Final Linearized System: V_{cmd} vs \omega');
xlabel('V_{cmd} [V]'); ylabel('\omega [rad/s]');
grid on;

figure(2); % Vc-Vcmd line (inverse f)
grid on; hold on;
plot(Vcmd, Vc_mapped, 'b');
xlabel('Vcmd [V]'); ylabel('Vc [V]');
title('Vc-Vcmd line (inverse f)');

% figure; % fitted Vo-Vc plot
% grid on; hold on;
% scatter(data_Vc,  data_omega,  60, 'b', 'filled', 'DisplayName', 'CW');
% plot(data_Vc, omega_fitted, 'm', 'linewidth', 2);
% xlabel('Vc [V]', 'fontsize', 13);   ylabel('\omega [rad/s]', 'fontsize', 13) ;
% title('Vo-Vc plot & fitting', 'fontsize', 12) ;
% legend('raw data', 'fitted line') ;
