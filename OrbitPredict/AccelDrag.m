% ��������㶯���ٶ�
% ���������
% PosVel - ����λ���ٶ�       dens - �����ܶȣ�������ʱ����ָ��ģ�ͼ��㣩
% ���������
% a - �����㶯������������ٶ�
function a = AccelDrag(PosVel,dens)
global CD s_m;
r = sqrt(PosVel(1)^2 + PosVel(2)^2 + PosVel(3)^2);
if nargin == 1
    dens = ComputeDenstiy(r);
end
w = 0.7292e-4;
v_rel = [PosVel(4) + w * PosVel(2);
         PosVel(5) - w * PosVel(1);
         PosVel(6)];
a = -0.5 * CD * s_m * 1e3 * dens * norm(v_rel) * v_rel;
end

%% ��������ܶȣ�ָ��ģ�ͣ�
% ���룺r - ���ǵ��ľ�   �����den - �����ܶ�
function den = ComputeDenstiy(r)
p0 = 3.6e-10;  H0 = 37.4;
r0 = 6408.4;   
H = H0 + 0.12 * (r - r0);                 
den = p0 * exp(-(r - r0) / H);
end

