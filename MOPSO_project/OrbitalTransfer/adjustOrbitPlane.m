% ��������
% ���룺coe - ��ʼ�������    dInc - �����ǵ���ֵ     dRaan - ������ྭ����ֵ
% �����T1��T2 - ����ʱ��    deltv1��deltv2 - �ٶ�����
% ����[T1, deltv1, T2, deltv2] = adjustOrbitPlane([6885;0.0001;40.5;10;20;5], 2, 5);
function [T1, deltv1, T2, deltv2] = adjustOrbitPlane(coe, dInc, dRaan, type)
global GM_Earth;
if nargin == 3
    type = 'twoBody';
end

coePlane = coe;   
coePlane(3) = AmendDeg(coePlane(3) + dInc,'0 - 180');
coePlane(4) = AmendDeg(coePlane(4) + dRaan,'-180 - 360');

[T1, x1, T2, x2] = yiMianTime(coe, coePlane);
[r_c,v_c] = Orbit_Element_2_State_rv(coe , GM_Earth);        
[r_t,v_t] = Orbit_Element_2_State_rv(coePlane , GM_Earth);   ht = cross(r_t, v_t);
if strcmp(type, 'twoBody')
    rvc = twoBodyOrbitRV([r_c;v_c], T1);
    v = cross(ht,x1)/norm(cross(ht,x1))*norm(ht)/norm(x1);
    deltv1 = v - rvc(4:6);
    
    rvc = twoBodyOrbitRV([r_c;v_c], T2);
    v = cross(ht,x2)/norm(cross(ht,x2))*norm(ht)/norm(x2);
    deltv2 = v - rvc(4:6);
end
end

