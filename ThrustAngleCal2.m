%% 方向矢量分解为俯仰角和方位角
% 输入：vec - 方向矢量
% 输出�Azimuth - ��λ��?(-180 - 360)     Elevation - ������?(-90 - 90)
function [Azimuth, Elevation] = ThrustAngleCal2(vec)  
z = [0;0;1];
x = [1;0;0];
Elevation = acosd(dot(vec,z));
if Elevation < 90
    Elevation = 90 - Elevation;
else
    Elevation = -Elevation + 90;
end

vec_z = [0;0;dot(vec,z)];
vec_xy = vec - vec_z;
Azimuth = acosd(dot(x,vec_xy)/norm(vec_xy));
end
