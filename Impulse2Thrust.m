% dll�����ӿ�
% ���룺
% ��ʼ������  rv_c
% ������Ϣ  p 4*n ÿһ��ǰ����Ϊ����������� ������Ϊ����ʩ��ʱ��
% ������С  Thrust_F
% ���������� mass
% �ȳ� Isp
% 
% �����
% ���ƿ��ػ�ʱ��  Thrust_t  2*n  ����ʱ�� �ػ�ʱ��
% ���ƽǶ�-VVLHϵ�� Thrust_angle
% ������ʣ������  Mass
function  [Thrust_t, Thrust_angle, Mass] = Impulse2Thrust(rv_c, p, Thrust_F, mass, Isp)
    ParamDefine;
    k = kCal(Thrust_F, Isp);
    Impulse_times = size(p, 2);
    [t_total, Thrust_angle, Mass] = MultiThrustOptimal2(Impulse_times, rv_c, p, Thrust_F, mass, k, Isp);
    Thrust_t = zeros(2, Impulse_times);
    for i = 1:Impulse_times
        Thrust_t(1,i) = p(4,i);
        Thrust_t(2,i) = Thrust_t(1,i) + t_total(i);
        if i == Impulse_times
            Thrust_t(1,i) = Thrust_t(1,i) - t_total(i);
            Thrust_t(2,i) = Thrust_t(2,i) - t_total(i);
        end
    end
    
end