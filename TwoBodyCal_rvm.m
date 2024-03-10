% RVm 前六项为RV 后面附加质量
% Thrust_f 为推力大小 推力大小恒定
% k 为质量变化率 一般为负数

function drvm = TwoBodyCal_rvm(t, RVm, Thrust_f, deg, k)   %RV惯性系下位置速度矢量  t时长
global GM_Earth;
Azimuth = deg(1);
Elevation = deg(2);

x = RVm(4);
y = RVm(5);
z = RVm(6);
m = RVm(7);

%%脉冲轨道系定向  需要求解对应时刻惯性系的脉冲
RotMat = (Inertial2Orbit(RVm))';
vec = [cosd(Elevation)*cosd(Azimuth); cosd(Elevation)*sind(Azimuth); sind(Elevation)];
accel = Thrust_f/m/1000;   % 根据推力计算加速度大小
r = norm(RVm(1:3));
dx = -GM_Earth/ r^3 * RVm(1) + accel * RotMat(1,:)* vec;
dy = -GM_Earth/ r^3 * RVm(2) + accel * RotMat(2,:)* vec;
dz = -GM_Earth/ r^3 * RVm(3) + accel * RotMat(3,:)* vec;

drvm = [x; y; z; dx; dy; dz; k];
end