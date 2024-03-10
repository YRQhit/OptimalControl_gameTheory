function [v]=velocity_cal(x,i,ra)
% VELOCITY_CAL is to calculate the velocity vector according to the
% position��inclination and RAAN
% x�������ڹ���ϵ�λ�ã������ǵ�λ��ʸ���� 
% i�� ������
% ra ��������ྭ

global GM_Earth;
% i=i/180*pi;
% ra=ra/180*pi;

% n = [sin(i)*cos(ra),sin(i)*sin(ra),cos(i)];                         %��õ�λ������
% dir_v=cross(n,x)/norm(cross(n,x));                                 %����ٶȷ���λʸ��
% 
% r= norm(x);
% v_val=sqrt(GM_Earth./r);
% v=v_val*dir_v;                                                      %����ٶ�ʸ��

Rx = [1 0 0;0 cosd(i) sind(i);0 -sind(i) cosd(i)];
Rz = [cosd(ra) sind(ra) 0;-sind(ra) cosd(ra) 0;0 0 1];
j = inv(Rz * Rx) * [0 0 1]';
r = norm(x);
k = x / r;
v = cross(j,k) * sqrt(GM_Earth/r);
end