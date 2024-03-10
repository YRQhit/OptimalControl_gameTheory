% 近距离上两星双脉冲机动对接
% 输入：coe_c:追踪星轨道六根数    coe_t：目标星轨道六根数     
%        distance:目标距离        t: 转移时间
% 输出：deltv1，deltv2 - 惯性系下两次速度脉冲（km/s)  
% 例：[deltv1, deltv2] = jiaoHuiDuiJie([6885;0;97.3;0;10;5], [6885;0;97.3;0;10;5.02], 0.3,2000);
function [deltv1, deltv2] = jiaoHuiDuiJie(coe_c, coe_t, distance, t)
global GM_Earth rad2deg;
n = sqrt(GM_Earth / coe_t(1)^3);
coe_t(6) = coe_t(6) + distance / coe_t(1) * rad2deg;

if nargin == 3
%     t = 1.1 * pi / n;
    t = pi / n;
end
% t = 3000;
[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
rvc = [r_c;v_c];
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
rvt = [r_t;v_t];
[delta,QXx]= Inertial2Relative(r_t,v_t,r_c,v_c);

[Qrr, Qrv, Qvr, Qvv] = CWStateTransitionMatrix(n, t);   
r0 = delta(1:3);             v0 = delta(4:6);
if abs(det(Qrv)) > 1e-3
    v1 = -inv(Qrv) * Qrr * r0;
else
    v1 = -pinv(Qrv) * Qrr * r0;
end
deltv1 = QXx' * (v1 - v0);
targetPosVel = OrbitPrediction(rvt, t , 60, [1 1],'RK7'); E = [];
for i = 1:10
    chasePosVel = OrbitPrediction([r_c;v_c + deltv1], t , 60, [1 1],'RK7');
    err = targetPosVel - chasePosVel;
    [delt,QXx2]= Inertial2Relative(targetPosVel(1:3),targetPosVel(4:6),chasePosVel(1:3),chasePosVel(4:6));
    E = [E norm(err(1:3))];
    if(norm(err(1:3)) < 0.0002)
        break;
    end
    
    if abs(det(Qrv)) > 1e-3
        v1 = v1 + inv(Qrv) * QXx2 * err(1:3); 
    else
        v1 = v1 + pinv(Qrv) * QXx2 * err(1:3); 
    end
    deltv1 = QXx' * (v1 - v0);
end
v2 = Qvr * r0 + Qvv * v1;
deltv2 = QXx2' * (-delt(4:6));
end

