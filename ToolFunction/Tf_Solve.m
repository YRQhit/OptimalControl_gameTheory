function T_f = Tf_Solve(coe_c , coe_t , Distance_B , JD , muu, day)
data = []; result = [];
[r_c_0 , ~] = Orbit_Element_2_State_rv(coe_c , muu);                          %׷���ǳ�ʼλ���ٶ�ʸ��(������)
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe_t , muu);                          %Ŀ���ǳ�ʼλ���ٶ�ʸ��(������)

thetaa_0 = acos(dot(r_c_0 , r_t_0) / (norm(r_c_0) * norm(r_t_0)));          %���ǳ�ʼ��ǲ�(rad)
a_c_0 = coe_c(1);                                                           %׷���ǳ�ʼ����볤��
a_t_0 = coe_t(1);                                                           %Ŀ���ǹ���볤��
a_c_1 = a_t_0 - Distance_B;                                                 %׷����ת�ƺ����볤��

w1 = 1 / sqrt(a_c_0 ^ 3 / muu);                      %׷���ǳ�ʼ������ٶ�
w2 = 1 / sqrt(a_c_1 ^ 3 / muu);                      %׷����Ŀ�������ٶ�
w0 = 1 / sqrt(a_t_0 ^ 3 / muu);                       %Ŀ���ǹ�����ٶ�

%% �������ת��ʱ��
[~,~,Tw] = Hm_transfer(a_c_0,a_c_1);
if a_t_0 > a_c_0 && sign(computeDegree(coe_c,coe_t)) == 1
    thetaa_0 = 2*pi - thetaa_0;
elseif a_t_0 < a_c_0 && sign(computeDegree(coe_c,coe_t)) == 1
    thetaa_0 = -thetaa_0;
elseif  a_t_0 < a_c_0 && sign(computeDegree(coe_c,coe_t)) == -1
    thetaa_0 = -2*pi + thetaa_0;
end
k = (w1 - w2) / (w0 - w2);
Tmin1 = (pi + thetaa_0 - Tw * w2) / (w0 - w2);
Tmin2 = (pi + thetaa_0 - Tw * omega_c_0) / (omega_t - omega_c_0);
t_0 = Tmin1 * (Tmin1 < Tmin2) + Tmin2 * (Tmin2 < Tmin1);
t_f = day * 86400;
% t_0 = thetaa_0 / (omega_c_0 - omega_t);                 % ���ʱ��
% t_f = thetaa_0/(omega_c_f - omega_t);                 % �ʱ��
% t_f = thetaa_0 / (omega_t - omega_c_f);    
t_0 = 0;
x_t_0 = [r_t_0' , v_t_0']';
x_t_0 = OrbitPrediction(x_t_0 ,t_0 , 300);             % Ŀ���ǹ�����Ƶ�����״̬
r_t_0 = x_t_0(1:3);         % Ŀ��������λ��ʸ��

T = 0;
% t_norm_vector_1_con = flagg * cross(r_t_0 , v_t_0) / norm(cross(r_t_0 , v_t_0)) ;   % Ŀ���ǹ����ʸ������
while T <= t_f   
    [thetaa_1,~] = IlluminationAngle(zeros(6,1),x_t_0,JD + T/86400);
    if a_c_1 > a_t_0
       thetaa_1 = 180 - thetaa_1;
    end
    data = [data,thetaa_1];                        % ̫�����սǱ�
    
    yy_temp = OrbitPrediction(x_t_0 ,60 , 60);    % Ŀ���ǵ���һ����
    r_t_0 = yy_temp (1 : 3);
    v_t_0 = yy_temp (4 : 6);
    x_t_0 = cat(1 , r_t_0 , v_t_0);
    T = T + 60;  
end
figure(1);       plot(data);   
for i = 2:length(data) - 1
    if (data(i) > data(i + 1))&&(data(i) > data(i - 1))
        result = [result,i];
    end
end
T_f = result * 60 + t_0;
end


