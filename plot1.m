clear;
% clc;
ParamDefine;
global GM_Earth
tic


Thrust_T = 300;
m = 1000;   %��������ʼ����3000kg

% �ٶ������̶� �������仯���ʲ��� m = m0 + kt
% ode���������дΪ[RV,m] ͬʱ����rv��m�ı仯

k = kCal(Thrust_T, 300);


coe_c = [6885;0.01;0.001;0;0;20];              
coe_t = [6875;0.01;0.001;0;0;25];
T = 3600;

[chase_r, chase_v] = Orbit_Element_2_State_rv(coe_c, GM_Earth);
[target_r, target_v] = Orbit_Element_2_State_rv(coe_t, GM_Earth);

rv_c = [chase_r; chase_v];                                 %��һ������ǰ��chaseRV
rv_t = [target_r; target_v];                               %��һ������ǰ��targetRV
%[x,~] = OrbitPrediction(rv_t,T,10,[0 0],'RK4');
% x = J2OrbitRV(rv_t, T);                                   %J2�㶯��Ŀ��Ư��25000s���λ��
[x,~] = OrbitPrediction(rv_t,T,60,[1 0],'RK7');
%[deltv1,deltv2] = lambertOptimal(rv_c, x, T);
[v1, v2, ~] = lambertIteration(rv_c', x(1:3)', T);         %����ʼĩ�ٶ�
% rv_c_start = [rv_c(1:3); v1'];
% coe_c_start = RV2COE(rv_c_start);
% rv_c_end = OrbitPrediction(rv_c_start,T,60,[1 1],'RK7');   
%rv_c_end = [x(1:3); v2'];
deltv1 = v1' - chase_v
deltv2 = x(4:6) - v2'


%% ����rv_c��ʼ ʩ��deltv1��deltv2�Ĺ켣

% p = cat(2, deltv1, deltv2);
% p = cat(1, p, [0 T])
% [t_total, Thrust_angle] = MultiThrustOptimal2(2, rv_c, p, 300, 3000, k)

p = cat(2, deltv1, deltv2)
p = cat(1, p, [0 T])
[t_total, Thrust_angle, pro_mass] = MultiThrustOptimal2(2, rv_c, p, 300, 500, k, 300)
toc





