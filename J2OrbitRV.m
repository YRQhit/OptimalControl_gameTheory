% J2������ټ���(������г��)
% ���� rv0 - ��ʼrv     T - ����ʱ��
% ��� rv - ����rv
function rv = J2OrbitRV(rv0, T)
global GM_Earth
coe0 = State_rv_2_Orbit_Element(rv0(1:3), rv0(4:6));
if coe0(2) < 0 || coe0(2) > 1
    err('六根数不正确');
end

coe = J2Orbit(coe0,T);
[r,v] = Orbit_Element_2_State_rv(coe, GM_Earth);
rv = [r;v];
end