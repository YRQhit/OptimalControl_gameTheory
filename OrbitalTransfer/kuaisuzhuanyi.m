% ����ת���㷨
% ���룺coe_c - ׷����������    coe_t - Ŀ����������   
%      distance - ����          startTime - ��ʼʱ��
% �������������
function [control] = kuaisuzhuanyi(coe_c,coe_t,distance,startTime)
global GM_Earth r_E rad2deg;
[rc,vc] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
q = distance / norm(rc);                                   % ����׷���ǳ�ǰĿ���ǵ����
[rt,vt] = Orbit_Element_2_State_rv([coe_t(1:5);coe_t(6) + q * rad2deg] , GM_Earth);

ha = coe_t(1) * (1 + coe_t(2)) - r_E;                      %Զ�ص������߶ȣ�km  35786
hp = coe_t(1) * (1 - coe_t(2)) - r_E;                      %���ص������߶ȣ�km
e = (ha - hp)/(r_E * 2+ ha + hp);         %ƫ����
we = sqrt(GM_Earth / coe_t(1)^3);         %�������н��ٶȣ�rad/s  2pi/86164

delta = Inertial2Relative(rt,vt,rc,vc);       %����ϵ������ϵ��ת��,�õ�������2����ں���1��λ���ٶ�ʸ���ں�����1�Ĺ��ϵ�µ�����
control.T = pi * (coe_t(1)^3/GM_Earth)^0.5;
x0 = delta(2);   z0 = -delta(1);   vz0 = -delta(4);xt = 0;
a = ((x0-xt)^2+4*z0^2)/2/abs(x0-xt);
control.deltv1 = [0;0;0];
if x0 == 0
    control.deltv1(3) = 0;
else
    control.deltv1(3) = -(we*(x0-xt)^2/4-we*z0^2)/(x0-xt)-vz0;
end
L_oi = Inertial2Orbit([rc;vc]);

targetPosVel = OrbitPrediction([rt;vt],control.T,60,[1,1],'RK7',startTime);
Kp1 = 1 / 5000; er1 = 0;er = 0;
Kp2 = 1 / 5000; E = [];
optimal.er = 100;
optimal.dvx = 0;
optimal.e = 100;
optimal.dv1z = control.deltv1(3);
j = 1;
for i = 1:30
    while j < 30
        v = L_oi' * control.deltv1;
        chasePosVel = OrbitPrediction([rc;vc + v],control.T,60,[1,1],'RK7',startTime); 
        er = norm(chasePosVel(1:3)) - norm(targetPosVel(1:3));
        if j == 1
            e1 = er;
        elseif j == 2 && abs(er) > abs(e1)        % ȷ���ȶ���
            Kp1 = 0.8 * Kp1;
            control.deltv1(1) = 0;
            j = 1;
            continue;
        end
        if abs(er) < 0.05            % ����
            break;
        end
        
        if abs(er) < optimal.er      % ����
            optimal.er = abs(er);
            optimal.dvx = control.deltv1(1);
        end
        control.deltv1(1) = control.deltv1(1) - Kp1 * er;
        j = j + 1;
    end
    j = 1;
    control.deltv1(1) = optimal.dvx;
    e =  computeVelDistance(chasePosVel, targetPosVel);E = [E,e];
    if abs(e) < 0.05             % ����
        break; 
    end
    
    if abs(e) < optimal.e       % ����
        optimal.e = abs(e);
        optimal.dv1z = control.deltv1(3);
    end
    control.deltv1(3) = control.deltv1(3) - Kp2 * e;
end
control.deltv1(3) = optimal.dv1z;
control.deltv1 =  L_oi' * control.deltv1;
[delta,QXx] = Inertial2Relative(targetPosVel(1:3),targetPosVel(4:6),chasePosVel(1:3),chasePosVel(4:6));
control.deltv2 = -QXx' * delta(4:6);
end

%%
function  velDistance = computeVelDistance(chasePosVel,targetPosVel) 
velDistance =  dot(chasePosVel(1:3) - targetPosVel(1:3),targetPosVel(4:6))/ norm(targetPosVel(4:6));
end
