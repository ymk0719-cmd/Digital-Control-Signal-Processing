%% 4. 모든 Step 세그먼트 오버레이 Plot (0~3초 기준으로 겹쳐서)
step_segs = find(vcmd_ref(seg_starts) ~= 0);

figure('Name', 'Step Overlay', 'Position', [100 550 800 500]);
hold on;

colors = lines(length(step_segs));  % 세그먼트마다 다른 색

for i = 1:length(step_segs)
    k   = step_segs(i);
    idx = seg_starts(k) : seg_ends(k);
    t_k = time(idx) - time(seg_starts(k));   % 0 기준 상대 시간

    % 3초 이내 데이터만 사용
    mask = t_k <= 3.0;

    plot(t_k(mask), omega(idx(mask)), ...
        'Color', colors(i,:), 'LineWidth', 1.0, ...
        'DisplayName', sprintf('Seg%d Vcmd=%.2fV', k, vcmd_ref(seg_starts(k))));
end

% omega_target 기준선 — 첫 번째 step 세그먼트 기준으로 표시
k1  = step_segs(1);
idx1 = seg_starts(k1) : seg_ends(k1);
t_k1 = time(idx1) - time(seg_starts(k1));
mask1 = t_k1 <= 3.0;
plot(t_k1(mask1), omega_target(idx1(mask1)), 'k--', 'LineWidth', 1.8, ...
    'DisplayName', '\omega_{target} (ref)');

xlabel('t [s]');
ylabel('\omega [rad/s]');
title('All Step Segments Overlaid (0–3 s window)');
legend('Location', 'best', 'FontSize', 8);
grid on;
xlim([0 3]);
hold off;