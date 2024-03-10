function [Delta_v_1 , Delta_v_2 , T1 , T2 , Tw] = banfei_Hohmann(coe_c , coe_t, Distance_f)

Distance_B = 0;                            %����ĩ��ʱ�̾���Լ��(km)

muu = 398600.4415;                          %��������������kg^3/s^2��
[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , muu);                          %׷���ǳ�ʼλ���ٶ�ʸ��(������)
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , muu);                          %Ŀ���ǳ�ʼλ���ٶ�ʸ��(������)

thetaa_rev = asin(Distance_f/2 / coe_t(1))*2;            %�ҷ�����Ŀ���Ǻ󷽽Ƕ�
thetaa_0 = acos(dot(r_c_0 , r_t_0) / (norm(r_c_0) * norm(r_t_0)));          %���ǳ�ʼ��ǲ�(rad)
thetaa_0 = thetaa_0 - thetaa_rev;

a_t_0 = coe_t(1);                           %Ŀ���ǹ���볤��
a_c_0 = coe_c(1);                           %׷���ǳ�ʼ����볤��
a_c_f = a_t_0 - Distance_B;                 %׷����Ŀ�����볤��
a_c_t = (a_c_0 + a_c_f)/2;                  %׷����ת�ƹ���볤��

T_c_0 = 2 * pi * sqrt(a_c_0 ^ 3 / muu);     %׷���ǳ�ʼ�������
T_c_t = 2 * pi * sqrt(a_c_t ^ 3 / muu);     %׷����ת�ƹ������
T_c_f = 2 * pi * sqrt(a_c_f ^ 3 / muu);     %׷����Ŀ��������
T_t = 2 * pi * sqrt(a_t_0 ^ 3 / muu);       %Ŀ���ǹ������

omega_c_0 = 2 * pi / T_c_0;                 %׷���ǳ�ʼ������ٶ�
omega_c_f = 2 * pi / T_c_f;                 %׷����Ŀ�������ٶ�
omega_t = 2 * pi / T_t;                     %Ŀ���ǹ�����ٶ�
%----------�Ż������ĳ�ʼ�²�-----------------%
Tw = T_c_t / 2;                             %���ת��ʱ�䣨��ֵ��
% T2 = 1200;
T2 = 0;
% T1 = (-thetaa_0 + pi + omega_c_f * T2 - omega_t * (Tw + T2)) / (omega_t - omega_c_0);
T1 = round((thetaa_0 + omega_t * Tw - pi) / (omega_c_0 - omega_t));
Delta_v_2 = 0;Delta_v_1 = 0;
% v_c_t_1_before = sqrt(muu / a_c_0);                  %��һ�α��ǰ׷�����ٶȣ���Բ������㣩
% v_c_t_2_after = sqrt(muu / a_c_f);                   %�ڶ��α���׷�����ٶȣ���Բ������㣩
% 
% Delta_v_1 = v_c_t_1_before*(sqrt(2*a_c_f/a_c_0/(1+a_c_f/a_c_0))-1);           %Hohmann����һ���ٶ�����
% Delta_v_2 = v_c_t_2_after*(1-sqrt(2/(1+a_c_f/a_c_0)));                        %Hohmann���ڶ����ٶ�����
% 
% x_0 = cat(1 , T1);                                                      %�Ż�������ֵ
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

thetaa_rev = asin(Distance_f/2 / coe_t(1))*2;            %�ҷ�����Ŀ���Ǻ󷽽Ƕ�

% thetaa_f = acosd(dot(r_c_f , r_t_f) / (norm(r_c_f) * norm(r_t_f)))
thetaa_f = acos(dot(r_c_f , r_t_f) / (norm(r_c_f) * norm(r_t_f)))

% c = abs(thetaa_f) - 0.01;
% c = acosd(dot((r_t_f - r_c_f) , v_t_f) / (norm(r_t_f - r_c_f)*norm(v_t_f)))-90;
c = [];
ceq = thetaa_f - thetaa_rev
end