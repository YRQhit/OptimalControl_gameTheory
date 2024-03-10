function Rrange = fuelestimate(fuelmass,r0,minR)
% RRANGE ����Ԥ��ȼ�϶�Ӧ��������С�������������������С�ж���������
% fuelmass��ȼ��������r0�ǳ�ʼ����߶�
% 

global GM_Earth


if nargin ==2 
    minR = 6700;
end

dv= fuelmass2dv(fuelmass/4)/1000;       % dv ��λkm/s

param = (dv/sqrt(GM_Earth/r0)+1)^2;
r2 = r0*param/(2-param);
dr = r2 - r0;
Rrange = [r0-dr,r2];
if Rrange(1) < minR
    drcomp = minR- Rrange(1);
    Rrange(1) = minR;
    Rrange(2) = r2;
end
end