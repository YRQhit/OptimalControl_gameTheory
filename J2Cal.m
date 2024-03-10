function drv = J2Cal(t, RV, Thrust_f, deg)   %RV惯�?�系下位置�?�度矢量  t时长
global GM_Earth J2 r_E
Azimuth = deg(1);
Elevation = deg(2);

x = RV(1);
y = RV(2);
z = RV(3);

%%脉冲轨道系定�?  �?要求解对应时刻惯性系的脉�?
RotMat = (Inertial2Orbit(RV))';
vec = [cosd(Elevation)*cosd(Azimuth); cosd(Elevation)*sind(Azimuth); sind(Elevation)];

r = norm(RV(1:3));
dx = -GM_Earth * x/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * z^2 / r^2)) + Thrust_f * RotMat(1,:)* vec;
dy = -GM_Earth * x/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * z^2 / r^2)) * (y / x) + Thrust_f * RotMat(2,:)* vec;
dz = -GM_Earth * z / r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (3 - 5 * z^2 / r^2))+ Thrust_f * RotMat(3,:)* vec;

drv = [RV(4); RV(5); RV(6); dx; dy; dz];
end

