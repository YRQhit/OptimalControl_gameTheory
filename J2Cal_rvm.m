% RVm ǰ����ΪRV ���渽������ ����J2�㶯
% Thrust_f Ϊ������С ������С�㶨
% k Ϊ�����仯�� һ��Ϊ����

function drvm = J2Cal_rvm(t, RVm, Thrust_f, deg, k)   %RV����ϵ��λ���ٶ�ʸ��  tʱ��
global GM_Earth J2 r_E;
Azimuth = deg(1);
Elevation = deg(2);

x = RVm(4);
y = RVm(5);
z = RVm(6);
m = RVm(7);

%%������ϵ����  ��Ҫ����Ӧʱ�̹���ϵ������
RotMat = (Inertial2Orbit(RVm))';
vec = [cosd(Elevation)*cosd(Azimuth); cosd(Elevation)*sind(Azimuth); sind(Elevation)];
accel = Thrust_f/m/1000;   % ��������������ٶȴ�С
r = norm(RVm(1:3));
dx =  accel * RotMat(1,:)* vec - GM_Earth * RVm(1)/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * RVm(3)^2 / r^2));
dy =  accel * RotMat(2,:)* vec - GM_Earth * RVm(1)/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * RVm(3)^2 / r^2)) * (RVm(2)/RVm(1));
dz =  accel * RotMat(3,:)* vec - GM_Earth * RVm(3) / r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (3 - 5 * RVm(3)^2 / r^2));

drvm = [x; y; z; dx; dy; dz; k];
end