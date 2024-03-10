% dll函数接口
% 输入：
% 初始六根数  rv_c
% 脉冲信息  p 4*n 每一列前三行为三轴脉冲分量 第四行为脉冲施加时刻
% 推力大小  Thrust_F
% 航天器质量 mass
% 比冲 Isp
% 
% 输出：
% 化推开关机时刻  Thrust_t  2*n  开机时刻 关机时刻
% 化推角度-VVLH系下 Thrust_angle
% 航天器剩余质量  Mass
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