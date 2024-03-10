coe_c = [42166;0.001;0.01;0;0;10];                % 任务星
T = 2000;                                         % 机动时间
delta = [0.5;-0.2;0.15;0.0001;0;-0.0002];         % 若不进行机动。T时间下，目标将到达我方星的相对位置


VVLH2LVLH = [0 0 -1;1 0 0;0 -1 0];
r = VVLH2LVLH' * delta(1:3);   v = VVLH2LVLH' * delta(4:6); 
[deltv1, deltv2] = CWtransfer(coe_c, [r;v], T);              % 惯性系下两次脉冲


coe_t = twoBodyOrbit(coe_c, T);
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
rvc = twoBodyOrbitRV([r_c;v_c + deltv1],T);
rvc(4:6) = rvc(4:6) + deltv2;

[relState,~]= Inertial2Relative(r_t,v_t,rvc(1:3),rvc(4:6));
r = VVLH2LVLH * relState(1:3);                               
v = VVLH2LVLH * relState(4:6); 
delta - [r;v]                                     % 状态偏差


% coe_t = twoBodyOrbit(coe_c, T);
% [r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
% rvc = [r_c;v_c];
% [r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
% rvt = [r_t;v_t] + [0.5;-0.2;0.15;0.0001;0;-0.0002];
% [deltv1,deltv2] = lambertOptimal(rvc, rvt, T)

function [deltv1, deltv2] = CWtransfer(coe_c, delta, t)
global GM_Earth;
coe_t = coe_c;
n = sqrt(GM_Earth / coe_t(1)^3);

if nargin == 2
    t = pi / n;
end

[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
rvt = [r_t;v_t];
[~,QXx]= Inertial2Relative(r_t,v_t,r_t,v_t);

[~, Qrv, ~, ~] = CWStateTransitionMatrix(n, t);   
if abs(det(Qrv)) > 1e-3
    v1 = inv(Qrv) * delta(1:3);
else
    v1 = pinv(Qrv) * delta(1:3);
end
deltv1 = QXx' * v1;

targetPosVel = OrbitPrediction(rvt, t , 60, [0 0],'RK7'); E = [];
for i = 1:10
    chasePosVel = OrbitPrediction([r_c;v_c + deltv1], t , 60, [0 0],'RK7');
    [relState,QXx2]= Inertial2Relative(targetPosVel(1:3),targetPosVel(4:6),chasePosVel(1:3),chasePosVel(4:6));
    err = relState - delta;
    E = [E norm(err(1:3))];
    if(norm(err(1:3)) < 0.0001)
        break;
    end
    
    if abs(det(Qrv)) > 1e-3
        v1 = v1 - inv(Qrv) * err(1:3); 
    else
        v1 = v1 - pinv(Qrv) * err(1:3); 
    end
    deltv1 = QXx' * v1;
end

deltv2 = QXx2' * (delta(4:6) - relState(4:6));
end


