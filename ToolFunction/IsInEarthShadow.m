% �ж������Ƿ��ڵ�Ӱ��
% ���룺pos - ����λ��   time - ������
% �����1 - ��Ӱ��   0 - ��Ӱ��
function signal = IsInEarthShadow(pos,time)
global r_E;
earth2sun = CalAstroBodyPos(time + 2400000.5);             % ����ָ��̫��ʸ��
q = asin(r_E / norm(earth2sun));               % �ٽ�Ƕ�
d = sqrt(norm(earth2sun)^2 - r_E^2);           % �ٽ����

sun2target = earth2sun - pos;                  % Ŀ����ָ��̫��ʸ��
qreal = acos(dot(sun2target,earth2sun) / norm(sun2target) / norm(earth2sun));
if qreal > q || norm(sun2target) < d
    signal = 0;
else
    signal = 1;
end
end

