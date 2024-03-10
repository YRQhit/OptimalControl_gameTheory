% RVm ǰ����ΪRV ���渽������
% Thrust_f Ϊ������С ������С�㶨
% k Ϊ�����仯�� һ��Ϊ����

function drv = TwoBodyCal(t, RV, Thrust_f, deg)   %RV����ϵ��λ���ٶ�ʸ��  tʱ��
global GM_Earth;
Azimuth = deg(1);
Elevation = deg(2);

x = RVm(4);
y = RVm(5);
z = RVm(6);

%%������ϵ����  ��Ҫ����Ӧʱ�̹���ϵ������
RotMat = (Inertial2Orbit(RVm))';
vec = [cosd(Elevation)*cosd(Azimuth); cosd(Elevation)*sind(Azimuth); sind(Elevation)];

r = norm(RVm(1:3));
dx = -GM_Earth/ r^3 * RVm(1) + Thrust_f * RotMat(1,:)* vec;
dy = -GM_Earth/ r^3 * RVm(2) + Thrust_f * RotMat(2,:)* vec;
dz = -GM_Earth/ r^3 * RVm(3) + Thrust_f * RotMat(3,:)* vec;

drv = [x; y; z; dx; dy; dz; k];
end