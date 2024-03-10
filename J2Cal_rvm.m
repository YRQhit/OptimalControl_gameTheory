% RVm 前六项为RV 后面附加质量 考虑J2摄动
% Thrust_f 为推力大小 推力大小恒定
% k 为质量变化率 一般为负数

function drvm = J2Cal_rvm(t, RVm, Thrust_f, deg, k)   %RV惯性系下位置速度矢量  t时长
global GM_Earth J2 r_E;
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
dx =  accel * RotMat(1,:)* vec - GM_Earth * RVm(1)/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * RVm(3)^2 / r^2));
dy =  accel * RotMat(2,:)* vec - GM_Earth * RVm(1)/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * RVm(3)^2 / r^2)) * (RVm(2)/RVm(1));
dz =  accel * RotMat(3,:)* vec - GM_Earth * RVm(3) / r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (3 - 5 * RVm(3)^2 / r^2));

drvm = [x; y; z; dx; dy; dz; k];
end