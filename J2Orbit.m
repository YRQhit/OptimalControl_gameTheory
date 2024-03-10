% J2������ټ���(������г��)
% ���� coe0 - ��ʼ�������     T - ����ʱ��
% ��� coe - ���չ������
function coe = J2Orbit(coe0, T)
global GM_Earth J2 r_E rad2deg;
a = coe0(1);  e = coe0(2);   i = coe0(3);
q = true2Mean(coe0(2),coe0(6));

p = a*(1-e^2);     n = sqrt(GM_Earth / a^3);
k1 = -1.5*J2*(r_E/p)^2*n*cosd(i);
k2 = 1.5*J2*(r_E/p)^2*n*(2-2.5*sind(i)^2);
k3 = n + 1.5*J2*(r_E/p)^2*n*(1-1.5*sind(i)^2)*sqrt(1-e^2);

coe = coe0;    
coe(4) = AmendDeg(coe0(4) + k1 * T * rad2deg, '0 - 360');
coe(5) = AmendDeg(coe0(5) + k2 * T * rad2deg, '0 - 360');
coe(6) = AmendDeg(q + k3 * T * rad2deg, '0 - 360');
coe(6) = mean2True(e, coe(6));
end