% RVm ǰ����ΪRV ���渽������
% Thrust_f Ϊ������С ������С�㶨
% k Ϊ�����仯�� һ��Ϊ����

function drvm = TwoBodyCal_rvm(t, RVm, Thrust_f, deg, k)   %RV����ϵ��λ���ٶ�ʸ��  tʱ��
global GM_Earth;
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
dx = -GM_Earth/ r^3 * RVm(1) + accel * RotMat(1,:)* vec;
dy = -GM_Earth/ r^3 * RVm(2) + accel * RotMat(2,:)* vec;
dz = -GM_Earth/ r^3 * RVm(3) + accel * RotMat(3,:)* vec;

drvm = [x; y; z; dx; dy; dz; k];
end