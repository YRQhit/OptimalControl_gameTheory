% 大范围转移：采用两次霍曼四次喷气，前三次沿速度方向，最后一次刹车脉冲保证两星伴飞
% 算法采用双闭环控制，外环通过调节转移轨道的飘飞时间改变相角，内环通过调节第二次霍曼
% 转移的脉冲和时间改变轨高。
% 输入：coe_c - 追踪星六根数       coe_t - 目标星六根数        T - 任务时间   
%   distance - 设定两星距离（追踪星在前为正）      startTime - 任务开始时间
% 输出：control - 机动策略           result - 控制结果
% 例：[control,result] = dafanwei([7000;0;0;0;0;0],[7000;0;0;0;0;20], 15000, 10, [2019 1 1 0 0 0])
% [control,result] = dafanwei([6885;0.0001;97.3;0;0;10],[6885;0.00012;97.3;0;0;22], 15000, 5, [2019 1 1 0 0 0])
function [control,result] = dafanwei(coe_c,coe_t, T, distance,startTime)
global GM_Earth rad2deg deg2rad;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % 追踪星初始位置速度
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % 目标星初始位置速度
control.deltv0 = 0;

%% 修正扁率
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
    q2 = distance / r2;                                   % 最终追踪星超前目标星的相角
    q = acos(dot(r_c0, r_t0) / norm(r_c0) / norm(r_t0));
    if sign(dot(r_c0 - r_t0,v_c0)) > 0                    % 追击相角方向判断
        q = -q;
    end
    q = q + q2;
end
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
%     [control.T1, control.Tw1, control.Tw2] = extcase(r0,8300,r2,q);
%     result.r1 = 8300;
end
%% 修正摄动导致的相角差
esum = 0;      result.E = [];       qOptimal = pi;                                     % 控制器系数
Kp = 50;    Ki = 0;       
eq = q2;    eq1 = 10;
veldistance = zeros(50);    
i = 1;    optimal.r1 = result.r1;
while 1
    [control.deltv1, control.deltv2, control.Tw1] = Hm_transfer(r0, result.r1);
    [control.deltv3, control.deltv4, control.Tw2] = Hm_transfer(result.r1, r2);
    control.T1 = T - control.Tw1 - control.Tw2;
    
    chasePosVel = OrbitPrediction([r_c0;v_c0 + control.deltv1 * v_c0 / norm(v_c0)], control.Tw1 , 60, [1,1],'RK7',startTime);                 % 追踪星第一段转移后位置速度
    chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv2;
    
    chasePosVel = OrbitPrediction(chasePosVel, control.T1 ,60, [1,1],'RK7',AddTime(startTime,control.Tw1));                                   % 追踪星飘飞后位置速度
    targetPosVel = OrbitPrediction([r_t0;v_t0], control.Tw1 + control.T1 ,60, [1,1],'RK7',startTime);                                         % 目标星最后一段转移前位置速度
    
    if abs(eq / q2) > 1 
          chasePosVel(4:6) = chasePosVel(4:6) + chasePosVel(4:6) / norm(chasePosVel(4:6)) * control.deltv3;
          chasePosVel = OrbitPrediction(chasePosVel, control.Tw2 ,60, [1,1],'RK7',AddTime(startTime,control.Tw1 + control.T1));               % 追踪星第二段转移后位置速度
          targetPosVel = OrbitPrediction(targetPosVel, control.Tw2 ,60, [1,1],'RK7',AddTime(startTime,control.Tw1 + control.T1)); 
          e = chasePosVel(1:3) - targetPosVel(1:3);            err = norm(e);   
          veldistance(i) = computeVelDistance(chasePosVel,targetPosVel);                                                      % 两星相对位置
          angle = acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));           % 两星夹角，rad
          direction = sign(dot(e,chasePosVel(4:6)));                                                                          % 角度方向
          qreal = direction * angle;
    else
          [control.Tw2,control.deltv3,control.deltv4, err, qreal, chasePosVel, targetPosVel] = excursionAdjustment(chasePosVel,targetPosVel,AddTime(startTime,control.Tw1 + control.T1));
          veldistance(i) = computeVelDistance(chasePosVel,targetPosVel);   
    end
    result.distance = err;                                         % 最终距离
    result.E = [result.E result.distance];
    result.angle = qreal * rad2deg;                                % 最终相角
    
   if abs(qreal - q2) < abs(qOptimal - q2)                         % 最优寄存
        optimal.distance = result.distance;  optimal.deltv1 = control.deltv1;
        optimal.angle = result.angle;        qOptimal = qreal;
        optimal.chasePosVel = chasePosVel;   optimal.targetPosVel = targetPosVel;
        optimal.deltv4 = control.deltv4;     optimal.Tw2 = control.Tw2;
        optimal.deltv3 = control.deltv3;     optimal.deltv2 = control.deltv2;
        optimal.r1 = result.r1;
   end
    
    eq = q2 - qreal;
    if i == 1
        eq1 = eq;
    end
    
    if abs(eq) > abs(eq1) 
        Kp = Kp / 2;
%         result.r1 = optimal.r1;
        eq = sign(eq) * abs(eq1);
    else
        i = i + 1;
    end
    
    if abs(err - abs(distance)) < 0.5  || i == 50                  % 死区
        break;
    elseif abs(eq / q2) < 2
        Ki = 0.2; esum = esum + eq;
    end
    result.r1 = result.r1 - Kp * eq - Ki * esum;
end
% plot(veldistance)
%% 填写输出结果
result.distance = optimal.distance;     result.angle = optimal.angle;
control.Tw2 = optimal.Tw2;
control.deltv1 = optimal.deltv1;      control.deltv2 = optimal.deltv2;
control.deltv3 = optimal.deltv3;      control.deltv4 = optimal.deltv4;
result.T = control.T1 + control.Tw1 + control.Tw2;
control.deltv = abs(norm(control.deltv0)) + abs(control.deltv1) + ...
    abs(control.deltv2) + abs(control.deltv3) + abs(norm(control.deltv4));

optimal.chasePosVel(4:6) = optimal.chasePosVel(4:6) + control.deltv4;
result.finalStateChase = optimal.chasePosVel;
result.finalStateTarget = optimal.targetPosVel;
result.coe_t = State_rv_2_Orbit_Element(result.finalStateTarget(1:3),result.finalStateTarget(4:6));
result.coe_c = State_rv_2_Orbit_Element(result.finalStateChase(1:3),result.finalStateChase(4:6));
result.velDistance = dot(optimal.chasePosVel(1:3) - optimal.targetPosVel(1:3),optimal.targetPosVel(4:6))...
    / norm(optimal.targetPosVel(4:6));
end
%% 计算速度方向距离
function  velDistance = computeVelDistance(chasePosVel,targetPosVel) 
velDistance =  dot(chasePosVel(1:3) - targetPosVel(1:3),targetPosVel(4:6))/ norm(targetPosVel(4:6));
end

%% 求解相角约束与时间约束方程
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

%% 根据过渡轨道计算最终位置
function [Tw,deltv1,deltv2,distance,qreal,chasePosVel,targetPosVel] = excursionAdjustment(rvc,rvt,startTime)
r1 = norm(rvc(1:3));   r2 = norm(rvt(1:3));  mark = 0; j = 1;
while mark == 0
    if r2 > r1
        [deltv1, ~, Tw] = Hm_transfer(r1, r2 + j);
    else
        [deltv1, ~, Tw] = Hm_transfer(r1, r2 - j);
    end
    v = rvc(4:6) + deltv1 * rvc(4:6) / norm(rvc(4:6));
    [~,chaseData] = OrbitPrediction([rvc(1:3);v], 1.1 * Tw , 60, [1,1],'RK7',startTime);
    [~,targetData] = OrbitPrediction(rvt, 1.1 * Tw , 60, [1,1],'RK7',startTime);
    num = length(chaseData);
    E = zeros(1,num);
    for i = 1:num
        E(i) = norm(chaseData(1:3,i)) - norm(targetData(1:3,i));
        if i ~= 1 &&  E(i) * E(i-1) < 0
            mark = 1;
            rvc(4:6) = v;
            break;
        end
    end
    j = j + 1;
end
Ttemp = 30;  A = E(i-1); B = E(i);
TwPre = 0;E = [];

%% 二分法确定同轨时刻
rvc = chaseData(:,i - 1);rvt = targetData(:,i - 1);
Tw = 60 * (i - 1) - 30;
for i = 1:10
    chasePosVel = OrbitPrediction(rvc, Ttemp, abs(Ttemp), [1,1],'RK7',startTime);
    targetPosVel = OrbitPrediction(rvt, Ttemp,abs(Ttemp), [1,1],'RK7',startTime);
    e = norm(chasePosVel(1:3)) - norm(targetPosVel(1:3));E = [E,e];
    if e * A < 0
        Ttemp = Ttemp - 30 / (2^i);
        B = e;
    elseif e * B < 0
        Ttemp = Ttemp + 30 / (2^i);
        A = e;
    else
        break;
    end
end
Tw = Tw + Ttemp;
%% 拟合方程确定同轨时刻
[delta,QXx]= Inertial2Relative(targetPosVel(1:3),targetPosVel(4:6),chasePosVel(1:3),chasePosVel(4:6));
deltv2 = -inv(QXx) * delta(4:6);

 e = chasePosVel(1:3) - targetPosVel(1:3);    distance = norm(e);                                               % 两星相对位置
 angle = acos(dot(chasePosVel(1:3),targetPosVel(1:3)) / norm(targetPosVel(1:3)) / norm(chasePosVel(1:3)));      % 两星夹角，rad
 direction = sign(dot(e,chasePosVel(4:6)));                                                                     % 角度方向
 qreal = direction * angle;
end


