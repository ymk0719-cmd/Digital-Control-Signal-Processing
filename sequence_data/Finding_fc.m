data = readmatrix('sequence_data/continuous_sequence_seg3s_20seg.out', ...
               'FileType','text', 'NumHeaderLines', 12);
time  = data(:,1);
omega = data(:,6);

% 예: 첫번째 스텝 구간 (0~3초) 잘라내기
mask = (time >=12) & (time <= 15);
t_step = time(mask);
w_step = omega(mask);

% 스텝 시작 시점 기준으로 재정렬
t_step = t_step - t_step(1);

% 63% 계산
w_final = mean(w_step(end-20:end));
target  = w_final * 0.632;
idx     = find(w_step >= target, 1, 'first');

tau = t_step(idx);
fc  = 1 / (2 * pi * tau);
fprintf('예상 fc = %.2f Hz\n', fc);