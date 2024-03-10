% RVm 前六项为RV 后面附加质量
% Thrust_f 为推力大小 推力大小恒定
% k 为质量变化率 一般为负数

function drv = TwoBodyCal(t, RV, Thrust_f, deg)   %RV惯性系下位置速度矢量  t时长
global GM_Earth;
Azimuth = deg(1);
Elevation = deg(2);

x = RVm(4);
y = RVm(5);
z = RVm(6);

%%脉冲轨道系定向  需要求解对应时刻惯性系的脉冲
RotMat = (Inertial2Orbit(RVm))';
vec = [cosd(Elevation)*cosd(Azimuth); cosd(Elevation)*sind(Azimuth); sind(Elevation)];

r = norm(RVm(1:3));
dx = -GM_Earth/ r^3 * RVm(1) + Thrust_f * RotMat(1,:)* vec;
dy = -GM_Earth/ r^3 * RVm(2) + Thrust_f * RotMat(2,:)* vec;
dz = -GM_Earth/ r^3 * RVm(3) + Thrust_f * RotMat(3,:)* vec;

drv = [x; y; z; dx; dy; dz; k];
end