% ���Լ���������λ���ת�Ƶ�deltv1����λkm/s) �� Tw����λs)
% r1Ϊ��ʼ����߶ȣ�r2ΪĿ�����߶�(��λkm)
function [deltv1,deltv2,Tw]= Hm_transfer(r1,r2)
global GM_Earth;
deltv1 = sqrt(GM_Earth / r1) * (sqrt((2*r2) / (r1 + r2)) - 1);
deltv2 = sqrt(GM_Earth / r2) * (1 - sqrt((2*r1) / (r1 + r2)));
r_avg = (r1+r2)/2;
Tw = pi*sqrt(r_avg^3/GM_Earth);
end