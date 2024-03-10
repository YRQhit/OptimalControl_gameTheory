% 近距离二体CW两脉冲转移
% 输入：coe_c - 任务星六根数                 coe_t - 目标星六根数 
%       delta - 转移点相对位置速度（LVLH）     t - 转移时间
% 输出：deltv1，deltv2 - 惯性系下的两次速度增量
% 例：[deltv1, deltv2] = CW2ImpulseTransfer([6885;0.001;97.5;10;20;5], [6885;0.0012;97.5;10;20;5.3], [10;0;0;0;0;0], 5000)
function [deltv1, deltv2] = CW2ImpulseTransfer(coe_c, coe_t, delta, t)
global GM_Earth;
n = sqrt(GM_Earth / coe_t(1)^3);

if nargin == 2
    t = pi / n;
end

[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
rvt = [r_t;v_t];
[p, QXx]= Inertial2Relative(r_t,v_t,r_c,v_c);

[Qrr, Qrv, ~, ~] = CWStateTransitionMatrix(n, t);   
if abs(det(Qrv)) > 1e-3
    v1 = inv(Qrv)*(delta(1:3) - Qrr*p(1:3));
else
    v1 = pinv(Qrv)*(delta(1:3) - Qrr*p(1:3));
end
deltv1 = QXx' * (v1 - p(4:6));

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
    deltv1 = QXx' * (v1 - p(4:6));
end

deltv2 = QXx2' * (delta(4:6) - relState(4:6));
end
