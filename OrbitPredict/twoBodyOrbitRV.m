% ���������ټ���
% ���룺rv0 - ��ʼ���λ���ٶ�      T - ����ʱ��
% �����rv - ���չ��λ���ٶ�
function rv = twoBodyOrbitRV(rv0, T)
coe0 = State_rv_2_Orbit_Element(rv0(1:3), rv0(4:6));
if coe0(2) < 0 || coe0(2) > 1
    err('����������ȷ');
end

coe = twoBodyOrbit(coe0,T);
[r,v] = Orbit_Element_2_State_rv(coe);
rv = [r;v];
end

