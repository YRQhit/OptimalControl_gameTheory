function [t_0, t_f] = T0_Tf_Solve(coe_c , coe_t , Distance_B , muu)

[r_c_0 , v_c_0] = Orbit_Element_2_State_rv(coe_c , muu);                    %׷���ǳ�ʼλ���ٶ�ʸ��(������)
[r_t_0 , ~] = Orbit_Element_2_State_rv(coe_t , muu);                    %Ŀ���ǳ�ʼλ���ٶ�ʸ��(������)

c_norm_vector = cross(r_c_0 , v_c_0) / (norm(r_c_0) * norm(v_c_0));                         %׷����ƽ�淨����
if dot(r_t_0 , c_norm_vector) >= 0
    r_t_0_projection2c = r_t_0 - dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%Ŀ���ǳ�ʼλ��ʸ����׷����ƽ��ͶӰ
else
    r_t_0_projection2c = r_t_0 + dot(r_t_0 , c_norm_vector) / norm(c_norm_vector) * c_norm_vector;%Ŀ���ǳ�ʼλ��ʸ����׷����ƽ��ͶӰ
end
thetaa_0 = acos(dot(r_c_0 , r_t_0_projection2c) / (norm(r_c_0) * norm(r_t_0_projection2c)));            %Ŀ���ǳ�ʼλ��ʸ��ͶӰ��׷����λ��ʸ���н�(rad)

a_c_0 = coe_c(1);                           %׷���ǳ�ʼ����볤��
a_t_0 = coe_t(1);
omega_c_0 = 1 / sqrt(a_c_0 ^ 3 / muu);                 %׷���ǳ�ʼ������ٶ�
omega_c_f = 1 / sqrt((a_t_0-Distance_B) ^ 3 / muu);    %׷����Ŀ�������ٶ�
% omega_c_f_1 = 1 / sqrt((a_c_0+coe_t(1)*coe_t(2)) ^ 3 / muu);

  
omega_t = 1 / sqrt(a_t_0 ^ 3 / muu);                     %Ŀ���ǹ�����ٶ�
% omega_t_1 = 1 / sqrt((a_t_0*(1+coe_t(2))) ^ 3 / muu);

t_0 = thetaa_0 / (omega_c_0 - omega_t);
t_f = thetaa_0/(omega_c_f - omega_t);

% t_0 = t_0 + 86400 * 0.05 * thetaa_0/pi*180;
% t_f = t_f-86400 * 0.25 * thetaa_0/pi*180;

end