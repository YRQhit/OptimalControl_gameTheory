% 抵近求解
% 输入：coe_c - 追踪星初始六根数      coe_t - 目标星初始六根数      
%       distance - 追踪星与目标星的最终距离       modelType - 轨道模型（二体/高精度）
% 输出：control - 变轨决策：
%                 Tw - 转移时长        T1 - 飘飞时长       deltv - 总的速度增量
%                 deltv1、deltv2 - 第一段变轨速度增量      
%       result - 变轨后结果数据：
%                 distance - 结束时两星距离     
%                 angle - 结束时两星相角       T - 转移总用时（在输入时长上下浮动）    
% ex：[control, result] = dijing([7000;0;0;0;0;0], [7500;0;0;0;0;30], 5, 'TwoBody')
%     [control, result] = dijing([7000;0;0;0;0;0], [7500;0;0;0;0;30], 5, 'HPOP')
function [control, result] = dijing(coe_c, coe_t, distance, modelType)
global GM_Earth rad2deg deg2rad;
if nargin == 3
    modelType = 'TwoBody';
end

if strcmp(modelType,'TwoBody')
    model = [0,0];
elseif strcmp(modelType,'HPOP')
    model = [1,1];
end

q1 = AmendDeg(coe_t(6) - coe_c(6));                       % 初始目标星超前追踪星的相角               
r0 = coe_c(1);       w0 = sqrt(GM_Earth / r0^3);          % 追踪星初始轨道半径与角速度            
r1 = coe_t(1);       w1 = sqrt(GM_Earth / r1^3);          % 目标星轨道半径与角速度
q2 = distance / r1;
q = q1 * deg2rad + q2;                                     % 正追击相角
temp = -sign(q) * (2 * pi - q);                            % 负追击相角

if abs(q) > abs(temp)                                     % 最优情况下采用小角度追击
    q = temp;
end

[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % 追踪星初始位置速度     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % 目标星初始位置速度

[control.deltv1, control.deltv2, control.Tw] = Hm_transfer(r0, r1);       % 二体下霍曼转移公式
control.T1 = (w1 * control.Tw + q - pi) / (w0 - w1);                      % 飘飞时长的解析解
while control.T1 < 0                                                      % 角度太小时无解，相角差加2pi
    s = sign(2*pi / (w0 - w1));
    control.T1 = control.T1 + s*2*pi/(w0 - w1);
end

result.T = control.T1 + control.Tw;                                       % 任务完成总时间
result.E = [];
%% 修正摄动导致的轨道波动（调整变轨速度增量）
if strcmp(modelType,'HPOP')                                               
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.T1 ,60,model,'RK7');
    targetPosVel = OrbitPrediction([r_t0;v_t0], result.T ,60,model,'RK7');
    [control.deltv1, control.deltv2, ~, ~] = Hm_Iteration(chasePosVel, norm(targetPosVel(1:3)));
end
control.deltv = abs(norm(control.deltv1)) + abs(norm(control.deltv2));

%% 修正摄动导致的相角差（调整飘飞时长）
for i = 1:30
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.T1 ,60,model,'RK7');
    chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv1;
    chasePosVel = OrbitPrediction(chasePosVel, control.Tw ,60,model,'RK7');  
    chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
        
    targetPosVel = OrbitPrediction([r_t0;v_t0], result.T ,60,model,'RK7');
    e = chasePosVel(1:3) - targetPosVel(1:3);        
    direction = sign(dot(e,chasePosVel(4:6)));
    q2real = direction * acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));
    
    result.distance = norm(e);            result.E = [result.E result.distance];
    result.angle = q2real * rad2deg;
    if strcmp(modelType,'TwoBody')
        break;
    end
    
    if  abs(q2real - q2) * r1 < 0.5
        break;
    else
        eq = q2 - q2real;                                 % 控制算法
        control.T1 = control.T1 + eq / (w0 - w1);
        result.T = control.T1 + control.Tw;
    end  
end
result.coe_c = State_rv_2_Orbit_Element(chasePosVel(1:3),chasePosVel(4:6),GM_Earth);
result.coe_t = State_rv_2_Orbit_Element(targetPosVel(1:3),targetPosVel(4:6),GM_Earth);
result.velDistance = dot(chasePosVel(1:3) - targetPosVel(1:3),targetPosVel(4:6)) / norm(targetPosVel(4:6));
end

