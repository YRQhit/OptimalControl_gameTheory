function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw] = banfei_Hohmann(coe_c , coe_t, Distance_f)

Distance_B = 0;                            %两星末端时刻距离约束(km)

muu = 398600.4415;                          %地球引力常量（kg^3/s^2）
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , muu);                          %追踪星初始位置速度矢量(行向量)
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , muu);                          %目标星初始位置速度矢量(行向量)

thetaa_rev = asin(Distance_f/2 / coe_t(1))*2;            %我方星在目标星后方角度
thetaa_0 = acos(dot(r_c_0 , r_t_0) / (norm(r_c_0) * norm(r_t_0)));          %两星初始相角差(rad)
thetaa_0 = thetaa_0 - thetaa_rev;

a_t_0 = coe_t(1);                           %目标星轨道半长轴
a_c_0 = coe_c(1);                           %追踪星初始轨道半长轴
a_c_f = a_t_0 - Distance_B;                 %追踪星目标轨道半长轴
a_c_t = (a_c_0 + a_c_f)/2;                  %追踪星转移轨道半长轴

T_c_0 = 2 * pi * sqrt(a_c_0 ^ 3 / muu);     %追踪星初始轨道周期
T_c_t = 2 * pi * sqrt(a_c_t ^ 3 / muu);     %追踪星转移轨道周期
T_c_f = 2 * pi * sqrt(a_c_f ^ 3 / muu);     %追踪星目标轨道周期
T_t = 2 * pi * sqrt(a_t_0 ^ 3 / muu);       %目标星轨道周期

omega_c_0 = 2 * pi / T_c_0;                 %追踪星初始轨道角速度
omega_c_f = 2 * pi / T_c_f;                 %追踪星目标轨道角速度
omega_t = 2 * pi / T_t;                     %目标星轨道角速度
%----------优化变量的初始猜测-----------------%
Tw = T_c_t / 2;                             %轨道转移时间（定值）
% T2 = 1200;
T2 = 0;
% T1 = (-thetaa_0 + pi + omega_c_f * T2 - omega_t * (Tw + T2)) / (omega_t - omega_c_0);
T1 = round((thetaa_0 + omega_t * Tw - pi) / (omega_c_0 - omega_t));
Delta_v_2 = 0;Delta_v_1 = 0;
% v_c_t_1_before = sqrt(muu / a_c_0);                  %第一次变轨前追踪星速度（按圆轨道计算）
% v_c_t_2_after = sqrt(muu / a_c_f);                   %第二次变轨后追踪星速度（按圆轨道计算）
% 
% Delta_v_1 = v_c_t_1_before*(sqrt(2*a_c_f/a_c_0/(1+a_c_f/a_c_0))-1);           %Hohmann变轨第一次速度脉冲
% Delta_v_2 = v_c_t_2_after*(1-sqrt(2/(1+a_c_f/a_c_0)));                        %Hohmann变轨第二次速度脉冲
% 
% x_0 = cat(1 , T1);                                                      %优化变量初值
% 
% A = [];
% b = [];
% Aeq = [];
% beq = [];
% u_lb = cat(1 , 0);
% u_ub = cat(1 , inf);
% p = cat(1 , r_c_0' , v_c_0' , r_t_0' , v_t_0' , T_c_t , Delta_v_1 , Delta_v_2 , T2 , Distance_f , coe_t);
% 
% [output] = fmincon(@CostFun , x_0 , A , b , Aeq , beq , u_lb , u_ub , @ (x) OrbitNonlcon(p , x));
% 
% T1 = output(1);

end

function minJ = CostFun(x)

T1 = x(1);
minJ = -T1;

end


function [c , ceq] = OrbitNonlcon(p , x)

T1 = x(1);

r_c_0 = p(1 : 3 , :);
v_c_0 = p(4 : 6 , :);
r_t_0 = p(7 : 9 , :);
v_t_0 = p(10 : 12 , :);
T_c_t = p(13 , :);
Delta_v_1 = p(14);
Delta_v_2 = p(15);
T2 = p(16);
Distance_f = p(17);
coe_t(1) = p(18);


x_c_0 = [r_c_0 ; v_c_0];

[~ , yy1] = R_K_4 (@OrbitModelDiff , x_c_0 , 0 , T1 , 60);
r_c_1 = yy1 (1 : 3 , size (yy1 , 2));
v_c_1 = yy1 (4 : 6 , size (yy1 , 2));
v_c_1 = v_c_1 / norm(v_c_1) * (norm(v_c_1) + Delta_v_1);
x_c_1 = [r_c_1 ; v_c_1];


[~ , yy2] = R_K_4 (@OrbitModelDiff , x_c_1 , T1 , T1 + T_c_t / 2 , 60);
r_c_2 = yy2 (1 : 3 , size (yy2 , 2));
v_c_2 = yy2 (4 : 6 , size (yy2 , 2));
v_c_2 = v_c_2 / norm(v_c_2) * (norm(v_c_2) + Delta_v_2);
x_c_2 = [r_c_2 ; v_c_2];


[~ , yy3] = R_K_4 (@OrbitModelDiff , x_c_2 , T1 + T_c_t / 2 , T1 + T_c_t / 2 + T2 , 60);
r_c_f = yy3 (1 : 3 , size (yy3 , 2));

x_t_0 = [r_t_0 ; v_t_0];

[~ , yy4] = R_K_4 (@OrbitModelDiff , x_t_0 , 0 , T1 + T_c_t / 2 + T2 , 60);
r_t_f = yy4 (1 : 3 , size (yy4 , 2));
distance = norm(r_t_f - r_c_f)

thetaa_rev = asin(Distance_f/2 / coe_t(1))*2;            %我方星在目标星后方角度

% thetaa_f = acosd(dot(r_c_f , r_t_f) / (norm(r_c_f) * norm(r_t_f)))
thetaa_f = acos(dot(r_c_f , r_t_f) / (norm(r_c_f) * norm(r_t_f)))

% c = abs(thetaa_f) - 0.01;
% c = acosd(dot((r_t_f - r_c_f) , v_t_f) / (norm(r_t_f - r_c_f)*norm(v_t_f)))-90;
c = [];
ceq = thetaa_f - thetaa_rev
end