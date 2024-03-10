% 大范围转移求解
% 输入：coe_c - 追踪星初始六根数      coe_t - 目标星初始六根数       T - 总时长
%       distance - 追踪星与目标星的最终距离       modelType - 轨道模型（二体/高精度）
% 输出：control - 变轨决策 ：
%                 Tw1 - 第一段转移时长       Tw2 - 第二段转移时长
%                 T1 - 飘飞时长              deltv - 总的速度增量
%                 deltv1、deltv2 - 第一段变轨速度增量          deltv3、deltv4 - 第二段变轨速度增量
%       result - 变轨后结果数据：
%                 r1 - 转移轨道高度            distance - 结束时两星距离
%                 sign - 结果有效标志
%                 angle - 结束时两星相角       T - 转移总用时（在输入时长上下浮动）
%ex: [control, result] = dafanweizhuanyi([7000;0;0;0;0;0],
%                   [7500;0;0;0;0;30], 20000, 5, 'TwoBody')
%    [control, result] = dafanweizhuanyi([7000;0;0;0;0;0],
%                   [7500;0;0;0;0;30], 20000, 5, 'HPOP')
function [control, result] = dafanweizhuanyi(coe_c, coe_t, T, distance, modelType)
global GM_Earth rad2deg deg2rad;
%% 输入缺省
if nargin == 4
    modelType = 'TwoBody';
end

if strcmp(modelType,'TwoBody')
    model = [0,0];
elseif strcmp(modelType,'HPOP')
    model = [1,1];
end

[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % 追踪星初始位置速度
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % 目标星初始位置速度
w2 = sqrt(GM_Earth / coe_t(1)^3);                                         % 目标轨道角速度
control.deltv0 = 0;

%% 偏心率过大时修正为圆
if coe_c(2) > 0.0016                                                      % 偏心率过大时需修正为圆
    n = cross(r_c0,v_c0)/norm(cross(r_c0,v_c0));                          % 单位法向量
    v_ep = sqrt(GM_Earth/norm(r_c0));                                     % 期待的卫星速度值
    v_epdir = cross(n,r_c0)/norm(cross(n,r_c0));
    v = v_ep *v_epdir;
    control.deltv0 = v - v_c0;
    v_c0 = v;
end
if coe_c(5) == coe_t(5) && coe_c(4) == coe_t(4) && coe_c(2) == coe_t(2) && coe_t(3) == 0
    r0 = coe_c(1);                                  % 初始轨道高度
    r2 = coe_t(1);                                  % 目标轨道高度
    
    q1 = AmendDeg(coe_t(6) - coe_c(6));             % 初始目标星超前追踪星的相角
    q2 = distance / r2;                             % 最终追踪星超前目标星的相角
    q  = q1 * deg2rad + q2;                         % 正追击相角
    temp = -sign(q) * (2 * pi - q);                 % 负追击相角
    if abs(q) > abs(temp)                           % 最优情况下采用小角度追击
        q = temp;
    end
else
    r0 = norm(r_c0);
    r2 = norm(r_t0);
    q = acos(dot(r_c0, r_t0) / norm(r_c0) / norm(r_t0));
    if sign(dot(r_c0 - r_t0,v_c0)) > 0              % 追击相角方向判断
        q = -q;
    end
    q2 = distance / r2;                             % 最终追踪星超前目标星的相角
end

%% 各段时间二体求解
% [result.r1, control.T1, control.Tw1, control.Tw2] = iterationSolve(r0, r2, q, T, T0);
[result.r1 , control.T1, control.Tw1, control.Tw2] = solve(r0, r2, q, T);                    % fsolve求解

if control.T1 < 0                     % 不满足大范围转移条件
    disp("无法在规定时间内完成任务");
    result.sign = 0;
    control.deltv = inf;
    return;
else
    result.sign = 1;
end
if result.r1 <= 6650
    [control.T1, control.Tw1, control.Tw2] = extcase(r0,6650,r2,q);
    result.r1 = 6650;
elseif result.r1 > 8300
    [control.T1, control.Tw1, control.Tw2] = extcase(r0,8300,r2,q);
    result.r1 = 8300;
end
% [result.r1, control.T1, control.Tw1, control.Tw2] = iterationSolve(r0, r2, q, T, [control.Tw1 control.Tw2]);   % 修正初解

%% 计算变轨速度增量，修正摄动导致的轨道波动
T1temp = 0;     control.deltv1 = 0;
if strcmp(modelType,'HPOP')
    for i = 1:3                                                          % 在二体的基础上继续迭代提高精度
        qtemp = q - 2 * pi + control.Tw1 * w2 + control.Tw2 * w2;        % 飘飞段需追赶的相角
        w1 = qtemp / control.T1;                                         % 飘飞段与目标轨的角速度差
        result.r1 = (GM_Earth / (w1 + w2)^2)^(1/3);                      % 飘飞轨道高度
        
        [deltv1, control.deltv2, ~, control.Tw1] = Hm_Iteration([r_c0;v_c0], result.r1);
        control.deltv1 = control.deltv1 + deltv1;
        v_c0 = v_c0 + v_c0 / norm(v_c0) * deltv1;
        [chasePosVel,~] = OrbitPrediction([r_c0;v_c0], control.Tw1 , 60, model,'RK7');
        chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
        
        [chasePosVel,~] = OrbitPrediction(chasePosVel, control.T1 ,60, model,'RK7');
        [targetPosVel,~] = OrbitPrediction([r_t0;v_t0], control.Tw1 + control.Tw2 + control.T1 ,60, model,'RK7');
        [control.deltv3, control.deltv4, ~, control.Tw2] = Hm_Iteration(chasePosVel, norm(targetPosVel(1:3)));
        
        if abs(control.T1 - T1temp) < 1e-3
            break;                                       % 停止迭代
        else
            T1temp = control.T1;
        end
    end
elseif strcmp(modelType,'TwoBody')
    [control.deltv1, control.deltv2, ~] = Hm_transfer(r0, result.r1);
    [control.deltv3, control.deltv4, ~] = Hm_transfer(result.r1, r2);
    v_c0 = v_c0 + v_c0 / norm(v_c0) * control.deltv1;                          % 追踪星变轨后速度
end
w1 = sqrt(GM_Earth / result.r1^3);

%% 修正摄动导致的相角差
esum = 0;      result.E = [];       qOptimal = pi;                                     % 控制器系数
erec = [0;0];                 cnt = 0;        erecflag = 0;
for i = 1:50
    [chasePosVel,~] = OrbitPrediction([r_c0;v_c0], control.Tw1 , 60, model,'RK7');                                % 追踪星第一段转移后位置速度
    if strcmp(modelType,'HPOP')
        chasePosVel(4:6) = chasePosVel(4:6) + control.deltv2;
    else
        chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv2;
    end
    
    [chasePosVel,~] = OrbitPrediction(chasePosVel, control.T1 ,60, model,'RK7');                                   % 追踪星飘飞后位置速度
    [targetPosVel,~] = OrbitPrediction([r_t0;v_t0], control.Tw1 + control.Tw2 + control.T1 ,60, model,'RK7');      % 目标星最终位置速度
    
    chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv3;
    [chasePosVel,~] = OrbitPrediction(chasePosVel, control.Tw2 ,60, model,'RK7');                                 % 追踪星第二段转移后位置速度
    
    e = chasePosVel(1:3) - targetPosVel(1:3);                                                                      % 两星相对位置
    angle = acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));      % 两星夹角，rad
    direction = sign(dot(e,chasePosVel(4:6)));                                                                     % 角度方向
    qreal = direction * angle;
    
    result.distance = norm(e);                    % 最终距离
    result.E = [result.E result.distance];
    result.angle = qreal * rad2deg;               % 最终相角
    
    if abs(qreal - q2) < abs(qOptimal - q2)       % 最优寄存
        optimal.T1 = control.T1;             optimal.distance = result.distance;
        optimal.angle = result.angle;        qOptimal = qreal;
        optimal.chasePosVel = chasePosVel;   optimal.targetPosVel = targetPosVel;
    end
    
    if  abs(qreal - q2) * r2 < 0.2                % 死区
        break;
    else
        eq = q2 - qreal;                          % 控制算法
        esum = esum + eq;
        if i==1
            erec(1) = eq;
            erec(2) = eq;
        elseif i ==2
            erec(2) = eq;
        else 
            erec(1) = erec(2);
            erec(2) = eq;
        end
%         if esum >2 
%             esum = -1;
%         elseif esum < -2
%             esum =1;
%         end
        if abs(eq) / q2 > 0.3
            Kp = 20;Ki = 0;
            if sign(erec(1)) ~= sign(erec(2)) || erecflag ==1
                cnt = cnt +1;
                Kp=Kp/cnt;Ki = 0.02;
                if Kp < 3
                    Kp =3;
                end
                erecflag =1;
            end
        elseif abs(eq) / q2 > 0.1 
            Kp = 8; Ki = 0.2;
        else
            Kp = 2; Ki = 0.1;
        end
        control.T1 = control.T1 + sign(r2 - result.r1) * (Kp * eq / w1 + Ki * esum / w1);
    end
end

%% 填写输出结果
result.distance = optimal.distance;     result.angle = optimal.angle;
control.T1 = optimal.T1;
result.T = control.T1 + control.Tw1 + control.Tw2;
control.deltv = abs(norm(control.deltv0)) + abs(norm(control.deltv1)) + ...
    abs(norm(control.deltv2)) + abs(norm(control.deltv3)) + abs(norm(control.deltv4));
if strcmp(modelType,'HPOP')
    optimal.chasePosVel(4:6) = optimal.chasePosVel(4:6) + control.deltv4;
else
    optimal.chasePosVel(4:6) = optimal.chasePosVel(4:6) + ...
        optimal.chasePosVel(4:6) / norm(optimal.chasePosVel(4:6)) * control.deltv4;
end
result.finalStateChase = optimal.chasePosVel;
result.finalStateTarget = optimal.targetPosVel;
result.coe_t = State_rv_2_Orbit_Element(result.finalStateTarget(1:3),result.finalStateTarget(4:6));
result.coe_c = State_rv_2_Orbit_Element(result.finalStateChase(1:3),result.finalStateChase(4:6));
result.velDistance = dot(optimal.chasePosVel(1:3) - optimal.targetPosVel(1:3),optimal.targetPosVel(4:6))...
    / norm(optimal.targetPosVel(4:6));
end

%% 迭代法求解相角约束与时间约束方程,修正fsolve的初解
function [r1, T1, Tw1, Tw2] = iterationSolve(r0, r2, q, T, T0)
if nargin == 4
    [~, ~, Tw1] = Hm_transfer(r0, r2);              % 第一段转移时长初值
    Tw2 = Tw1;                                      % 第二段转移时长初值
elseif nargin == 5
    Tw1 = T0(1);
    Tw2 = T0(2);
end
global GM_Earth;

w2 = sqrt(GM_Earth / r2^3);                         % 目标星轨道角速度
T1 = T - Tw1 - Tw2;                                 % 飘飞时长初值
T1temp = 0;
for i = 1:30
    qtemp = q - 2 * pi + Tw1 * w2 + Tw2 * w2;                % 飘飞段需追赶的相角
    w1 = qtemp / T1;                                         % 飘飞段与目标轨的角速度差
    r1 = (GM_Earth / (w1 + w2)^2)^(1/3);                     % 飘飞轨道高度
    
    [~, ~, Tw1] = Hm_transfer(r0, r1);                    % 第一段转移时长
    [~, ~, Tw2] = Hm_transfer(r1, r2);                    % 第二段转移时长
    
    T1 = T - Tw1 - Tw2;                                   % 飘飞时长
    if abs(T1 - T1temp) < 1e-3
        break;                                            % 停止迭代
    else
        T1temp = T1;
    end
end
end

%% 求解相角约束与时间约束方程，用于替换主函数中的迭代法
function [r1 , T1, Tw1, Tw2] = solve(r0, r2, q, T)
global GM_Earth;
f =  @(r1)(2 * pi - T * sqrt(GM_Earth / r2^3) - q + ...
    (T - pi * sqrt((r0 + r1)^3 / 8 / GM_Earth) - ...
    pi * sqrt((r1 + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1^3));
r1 = fsolve(f,r0);
[~, ~, Tw1] = Hm_transfer(r0, r1);
[~, ~, Tw2] = Hm_transfer(r1, r2);
T1 = T - Tw1 - Tw2;
end




