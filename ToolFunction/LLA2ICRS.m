% ��γ��ת����ϵ
% ���룺Mjd - ����ʱ��  lonlan - ����/γ��/���
% �����pos - ����ϵ��λ��
function pos = LLA2ICRS(Mjd,lonlan)
global r_E;
f = 1 / 298.257223563;									% ������ʣ�������
R_p = (1 - f) * r_E;									% ���򼫰뾶
e1 = sqrt(r_E * r_E - R_p * R_p) / r_E;
lon = lonlan(1);lan = lonlan(2);alt = lonlan(3);
temp = r_E / sqrt(1 - e1 * e1 * sind(lan) * sind(lan));
pos = zeros(3,1);
pos(1) = (temp + alt)*cosd(lan)*cosd(lon);
pos(2) = (temp + alt)*cosd(lan)*sind(lon);
pos(3) = (temp * (1 - e1*e1) + alt)*sind(lan);

E = ICRS2ITRS(Mjd); 
pos = E' * pos;
end

