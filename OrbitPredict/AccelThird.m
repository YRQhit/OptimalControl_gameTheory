% ���������㶯���ٶ�
% ���룺Pos - �����ڵ��Ĺ���ϵ�µ�����          Mjd - ������
% �����a - �����㶯���ٶ�
function a = AccelThird(Pos , Mjd)
global GM_Sun  GM_Moon;  

[~,~,~,~,~,~,~,~,~,r_moon,r_sun] = JPL_Eph_DE405(Mjd + 66.184 / 86400);
r_moon = r_moon./1000;       r_sun = r_sun./1000;                          % ���Ĺ���ϵ�µ�����λ��ʸ��

erthToSun = (r_sun(1)^2 + r_sun(2)^2 + r_sun(3)^2)^1.5;                    % �յؾ���(���壩
earthToMoon = (r_moon(1)^2 + r_moon(2)^2 + r_moon(3)^2)^1.5;               % �µؾ���(���壩
 
R_sun = ((Pos(1) - r_sun(1))^2 + (Pos(2) - r_sun(2))^2 + (Pos(3) - r_sun(3))^2)^1.5;           % ������̫������
R_moon = ((Pos(1) - r_moon(1))^2 + (Pos(2) - r_moon(2))^2 + (Pos(3) - r_moon(3))^2)^1.5;       % �������������

a_sun = - GM_Sun * ((Pos -  r_sun) / R_sun + r_sun / erthToSun);           % ̫���㶯���ٶ�
a_moon = - GM_Moon * ((Pos -  r_moon) / R_moon + r_moon / earthToMoon);    % �����㶯���ٶ�
a = a_sun + a_moon;
end

