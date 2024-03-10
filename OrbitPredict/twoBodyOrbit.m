% ���������ټ���
% ���룺coe0 - ��ʼ�������      T - ����ʱ��
% �����coe - ���չ������
function coe = twoBodyOrbit(coe0,T)
global GM_Earth rad2deg;
q = true2Mean(coe0(2),coe0(6));
Tperiod = 2 * pi * sqrt(coe0(1)^3 / GM_Earth);
M = 2*pi / Tperiod * T * rad2deg;
q = AmendDeg(mean2True(coe0(2),AmendDeg(q + M,'0 - 360')),'0 - 360');
coe = coe0;  coe(6) = q;
end

