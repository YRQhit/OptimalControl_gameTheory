% ȼ�����ļ���
% ���룺dv - �ٶ�����(km/s)    m - ������������kg)
% �����fuel - ȼ������(kg)
function fuel = fuelCost(dv,m,I)
if nargin == 2
    I = 285;
end

g0 = 9.8; dv = abs(dv) * 1000;
fuel = m * (1 - exp(-dv / I / g0));
end

