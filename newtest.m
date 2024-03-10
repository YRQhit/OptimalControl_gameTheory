clear;
% clc;
ParamDefine;
global GM_Earth
tic
%推力加速度大小
Thrust_f = 0.0001;
Thrust_T = 300;
m = 3000;   %航天器初始质量3000kg

% 假定推力固定 则质量变化速率不变 m = m0 + kt
% ode积分输入改写为[RV,m] 同时计算rv和m的变化

k = kCal(Thrust_T);


coe_c = [42166;0.01;0.001;0;0;20];              
coe_t = [42164;0.01;0.001;0;0;25];
T = 25000;

[chase_r, chase_v] = Orbit_Element_2_State_rv(coe_c, GM_Earth);
[target_r, target_v] = Orbit_Element_2_State_rv(coe_t, GM_Earth);

rv_c = [chase_r; chase_v];                                 %第一次脉冲前的chaseRV
rv_t = [target_r; target_v];                               %第一次脉冲前的targetRV
%[x,~] = OrbitPrediction(rv_t,T,10,[0 0],'RK4');
% x = J2OrbitRV(rv_t, T);                                   %J2摄动下目标漂移25000s后的位置
[x,~] = OrbitPrediction(rv_t,T,60,[1 0],'RK7',[2022 9 9 0 0 0]);
%[deltv1,deltv2] = lambertOptimal(rv_c, x, T);
[v1, v2, ~] = lambertIteration(rv_c', x(1:3)', T, [2022 9 9 0 0 0]);         %计算始末速度
% rv_c_start = [rv_c(1:3); v1'];
% coe_c_start = RV2COE(rv_c_start);
% rv_c_end = OrbitPrediction(rv_c_start,T,60,[1 1],'RK7');   
%rv_c_end = [x(1:3); v2'];
deltv1 = v1' - chase_v
deltv2 = x(4:6) - v2'


%% 跟踪rv_c开始 施加deltv1、deltv2的轨迹

% p = cat(2, deltv1, deltv2);
% p = cat(1, p, [0 T])
% [t_total, Thrust_angle] = MultiThrustOptimal2(2, rv_c, p, 300, 3000, k)

p = cat(2, deltv1, deltv2, deltv1, deltv2)
p = cat(1, p, [0 T 2*T 3*T])
[t_total, Thrust_angle, pro_mass] = MultiThrustOptimal2(4, rv_c, p, 300, 3000, k)
toc

