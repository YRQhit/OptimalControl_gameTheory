function [v]=velocity_cal(x,i,ra)
% VELOCITY_CAL is to calculate the velocity vector according to the
% position，inclination and RAAN
% x是卫星在轨道上的位置（即卫星的位置矢量） 
% i是 轨道倾角
% ra 是升交点赤经

global GM_Earth;
% i=i/180*pi;
% ra=ra/180*pi;

% n = [sin(i)*cos(ra),sin(i)*sin(ra),cos(i)];                         %获得单位法向量
% dir_v=cross(n,x)/norm(cross(n,x));                                 %获得速度方向单位矢量
% 
% r= norm(x);
% v_val=sqrt(GM_Earth./r);
% v=v_val*dir_v;                                                      %获得速度矢量

Rx = [1 0 0;0 cosd(i) sind(i);0 -sind(i) cosd(i)];
Rz = [cosd(ra) sind(ra) 0;-sind(ra) cosd(ra) 0;0 0 1];
j = inv(Rz * Rx) * [0 0 1]';
r = norm(x);
k = x / r;
v = cross(j,k) * sqrt(GM_Earth/r);
end