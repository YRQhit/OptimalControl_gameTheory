% 共面双椭圆交会
% 输入：coe_c - A星六根数          coe_t - B星六根数       type - 交会方式
% 输出：control - 变轨量
%           deltv - 速度增量            T - B星飘飞时间             TJiaoHui - 交会时间
%       result - 交会结果量
%           distance - 交会最短距离     relativeVel - 交会相对速度   T2 - 交会周期
% ex:[control, result] = gongMianJiaoHui([6700;0;0;0;0;2], [6700;0;0;0;0;0], 'short')
function [control, result] = gongMianJiaoHui(coe_c, coe_t, type)
global GM_Earth;
r = coe_c(1);           q = coe_c(6) - coe_t(6);
if q < 0
    q = q + 360;
end

if nargin == 2
    type = 'both';
end

if strcmp(type, 'short')
    [q_a, c] = TrajectoryPlan(r, q, 'short');
elseif strcmp(type, 'long')
    [q_a, c] = TrajectoryPlan(r, q, 'long');

elseif strcmp(type, 'both')
      [control(1), result(1)] = gongMianJiaoHui(coe_c, coe_t, 'short');
      [control(2), result(2)] = gongMianJiaoHui(coe_c, coe_t, 'long');
      return;
end

control.T = (q + 180) / 360 * 2 * pi * sqrt(r^3 / GM_Earth);                % B星在圆轨道上飘飞时间
control.deltv = (sqrt(1 + c / q_a) - 1) * sqrt(GM_Earth / r) * 1000;        % 霍曼变轨速度增量,单位m
T1 = 2 * pi * sqrt(r^3 / GM_Earth);                                         % 圆轨道周期
result.T2 = 2 * pi * sqrt(q_a^3 / GM_Earth);                                % 椭圆轨道周期
k = 0.25 - asin((q_a - r) / q_a) / 2 / pi - (q_a - r) * sqrt(2 * q_a * r - r^2) / 2 / pi / q_a^2;
T3 = k * result.T2;                                                         % B星在椭圆轨道上飘飞时间

if strcmp(type, 'short')
    control.TJiaoHui = result.T2 - T3;                                              % 两星交会时间（存在舍入误差）
elseif strcmp(type, 'long')
    control.TJiaoHui = result.T2 + T3;  
end
    
%% 核算最短距离,相对速度
coe_t = coe_c;     coe_t(6) = coe_c(6) - q;
coe_c(1) = q_a;    coe_c(2) = c / q_a;
[r_c0 , v_c0] = Orbit_Element_2_State_rv(coe_c , GM_Earth);               % A星初始位置速度     
[r_t0 , v_t0] = Orbit_Element_2_State_rv(coe_t , GM_Earth);               % B星初始位置速度

while 1
    chasePosVel = OrbitPrediction([r_c0;v_c0], control.TJiaoHui ,60,[0 0],'RK7');       % 交会点A星的位置速度
    targetPosVel0 = OrbitPrediction([r_t0;v_t0], control.T ,60,[0 0],'RK7');
    targetPosVel0(4:6) = targetPosVel0(4:6) + targetPosVel0(4:6) / norm(targetPosVel0(4:6)) * control.deltv / 1000;
    targetPosVel = OrbitPrediction(targetPosVel0, control.TJiaoHui - control.T ,60,[0 0],'RK7');   % 交会点B星位置速度

    result.distance = norm(chasePosVel(1:3) - targetPosVel(1:3));                   % 两星交会最短距离
    result.relativeVel = norm(chasePosVel(4:6) - targetPosVel(4:6));                % 两星交会相对速度
    
    if result.distance > 0.1                                                        % 防止两星相撞（由于计算误差，两星距离通常不会为0）
       break; 
    else
        control.T = control.T + 0.1 * T1 / (r * 2 * pi); 
    end
end
end
%% 根据初始的相角及轨道半径计算交会椭圆
% 输入：r - 圆轨道半径          q - 初始时刻B星滞后A星的相角    type - 长周期交会/短周期交会 
% 输出：q_a - 交会椭圆半长轴    c - 交会椭圆焦距
function [q_a, c] = TrajectoryPlan(r, q, type)
a = r:5:r + 100;
if strcmp(type, 'short')
    y = ((pi + 2 .* asin((a - r) ./ a) + 2 * (a - r) .* sqrt(2 .* a * r - r^2) ./ a.^2)...
        .* (a ./ r).^1.5 - pi) * 180 / pi;                               % 根据轨道动力学得到的半长轴与相角关系（短周期交会）
elseif strcmp(type, 'long')
    y = ((pi - 2 * asin((a - r) ./ a) - 2 * (a - r) .* sqrt(2 .* a * r - r^2) ./ a.^2)... 
        .* (a ./ r).^1.5 - pi) * 180 / pi;                                % 根据轨道动力学得到的半长轴与相角关系（长周期交会）
end

x = [ones(length(a),1) a'];
param = pinv(x) * y';                                               % 一次函数拟合关系                             
q_a = (q - param(1)) / param(2);                                    % 由拟合关系得到的椭圆半长轴  
while 1
    if strcmp(type, 'short')
        y_q = ((pi + 2 * asin((q_a - r) / q_a) + 2 * (q_a - r) * sqrt(2 * q_a * r - r^2) / q_a^2)...
            * (q_a / r)^1.5 - pi) * 180 / pi;   
    elseif strcmp(type, 'long')
        y_q = ((pi - 2 * asin((q_a - r) / q_a) - 2 * (q_a - r) * sqrt(2 * q_a * r - r^2) / q_a^2)...
            * (q_a / r)^1.5 - pi) * 180 / pi;
    end
        
    e = y_q - q; 
    if e * r * (pi /180) < 5
        break;
    else
        q_a = q_a + (e - param(1)) / param(2);
    end
end
c = q_a - r;
end


















