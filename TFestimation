% ---------------------------------------------%
%         transfer function estimation         %
% ---------------------------------------------%

close all; clear all; clc ;

data = readmatrix("bode_result.out", 'FileType', 'text');

tblOmega    = 2 * pi * data(:,1);          % Hz → rad/s
tblMagAtt   = data(:,2);                   % [dB]
tblPhsDelay = data(:,3) * pi / 180;        % deg → rad

% ✅ 핵심 수정: dB → linear 변환
tblMagLin   = 10.^(tblMagAtt / 20);
tblFreqResp = tblMagLin .* exp(1j * tblPhsDelay);

Nnum = 0;
Nden = 2;
[num, den] = invfreqs(tblFreqResp, tblOmega, Nnum, Nden);
EstTF = tf(num, den);

figure; bode(EstTF); grid on;

