clc;
clear;
ParamDefine;

%% 输入参数
coe_c = [42166;0.001;0.01;0;0;10];                    % 任务星
coe_t = [42166;0.0012;0.01;0;0;10.02];                % 目标星
degree = 34;                                          % 光照角约束
Tmax = 30000;                                         % 时间约束
delta = [0;0;10;0;0;0];                               % 转移相对位置速度
startTime = [2022 1 1 0 0 0];                         % 任务开始时间

%% 光照约束
for i = 0:300:Tmax
    t = Tmax - i;
    coe = twoBodyOrbit(coe_t, t);
    [r_t,v_t] = Orbit_Element_2_State_rv(coe , GM_Earth);
    
    ICRF2VVLH = Inertial2Orbit([r_t;v_t]);
    r_c = r_t + ICRF2VVLH' * delta(1:3);
    [~,actualDegree] = IlluminationAngle(r_c,[r_t;v_t],startTime,t);           
    if actualDegree <= degree
        break;
    end
end

if actualDegree > degree
    disp('不满足光照约束');
    return;
end

%% 计算脉冲（输出：两次脉冲和转移时间）
VVLH2LVLH = [0 0 -1;1 0 0;0 -1 0];
r = VVLH2LVLH * delta(1:3);   v = VVLH2LVLH * delta(4:6); 
[deltv1, deltv2] = CW2ImpulseTransfer(coe_c, coe_t, [r;v], t);               % 惯性系下的两次脉冲

%% 制导精度
coe_t = J2Orbit(coe_t, t);
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);

[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
rvc = J2OrbitRV([r_c;v_c + deltv1],t);
rvc(4:6) = rvc(4:6) + deltv2;

[relState,~]= Inertial2Relative(r_t,v_t,rvc(1:3),rvc(4:6));
r = VVLH2LVLH' * relState(1:3);                               
v = VVLH2LVLH' * relState(4:6); 
err = norm(delta - [r;v]);                     % 制导精度



