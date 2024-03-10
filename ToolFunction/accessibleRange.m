% �ɴ��������
% ���룺fuel - ȼ������(kg)   m - ��������(kg)  
%      r1 - ��ʼ����߶�(km)  I - �ȳ�
% �����range - �ɴ�����Χ
function range = accessibleRange(fuel, m, r1, I)
global GM_Earth;
if nargin <= 3
    I = 285;
    if nargin == 2
        r1 = 42166;
    end
end
dv = fuelmass2dv(fuel,m,I,9.8,0.999) / 1000;
k = 2 / (0.5 * dv * sqrt(r1 / GM_Earth) + 1)^2 - 1;
range(2) = r1 / k;

k = 2 / (- 0.5 * dv * sqrt(r1 / GM_Earth) + 1)^2 - 1;
range(1) = r1 / k;
f = @(r2)(abs(sqrt(2*GM_Earth*r2/(r1^2+r1*r2)) - sqrt(GM_Earth / r1)) + abs(sqrt(GM_Earth / r2) - sqrt(2*GM_Earth*r1/(r2^2+r1*r2))) - dv);
fplot(f,[range(1) - 100 ,range(2) + 100]);
end

