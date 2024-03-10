%%  ��������Ż�����  ���Ǻ����������仯
% ���� ������� Thrust_num
%      ׷����RV rv_c ��һ������ʱ�� ����ʩ��ǰRV
%      ÿ�����������Ϣ Thrust_Impulsive 4*n ÿ��Ϊһ�������deltv �Լ���Ӧ����ʱ�� 
%      �������� Thrust_f
%      mass����������ʼ����   k�������仯�ʣ������κ����������㶨�Ҵ�С��ȣ�
%  
% ��� ÿ������ʩ��ʱ�� t_total 1*n
%      ÿ������ʩ�ӽǶȣ����ϵ�� Thrust_angle 2*n

function [t_total, Thrust_angle, pro_mass] = MultiThrustOptimal2(Thrust_num, rv_c, Thrust_Impulsive, Thrust_f, mass, k, Isp)
    t_total = zeros(1, Thrust_num);
    Thrust_angle = zeros(2, Thrust_num);
    E_rv = zeros(6, Thrust_num-1);             %ÿ��Ϊÿ������ʱ�̵�����RV��ʩ������ǰ��
    final_mass = mass;    %  
    % �������������������
    for i = 1:Thrust_num
        dm = fuelCost(norm(Thrust_Impulsive(1:3, i)), final_mass, Isp);
        final_mass = final_mass - dm;
    end
    % ��������RV
    cal_rv_c = rv_c;
    for i = 1:Thrust_num-1
        cal_rv_c = cal_rv_c + [0; 0; 0; Thrust_Impulsive(1:3, i)];   %����ʩ��������RV
       E_rv(:, i) = OrbitPrediction(cal_rv_c, Thrust_Impulsive(4, i+1)-Thrust_Impulsive(4, i), 60, [1 0], 'RK4');     % ��������    
%        E_rv(:,i) = J2OrbitRV(cal_rv_c, Thrust_Impulsive(4, i+1)-Thrust_Impulsive(4, i));                              % J2����                                        
%         E_rv(:, i) = OrbitPrediction(cal_rv_c, Thrust_Impulsive(4, i+1)-Thrust_Impulsive(4, i), 60, [1 1], 'RK7');      % �߾���
        cal_rv_c = E_rv(:, i);                                       
    end
    
    % ���ն�����RV����
    rv_calangel = E_rv(:, end);
    E_rv(:, end) = E_rv(:, end) + [0; 0; 0; Thrust_Impulsive(1:3, end)];
    RVEnd = E_rv(:, end);
    
    FinalTrip_t = Thrust_Impulsive(4, Thrust_num)-Thrust_Impulsive(4, Thrust_num-1);  % ���ն�ת��ʱ��
    FinalImpulse_t = norm(Thrust_Impulsive(1:3, Thrust_num))/(Thrust_f/final_mass)*1000;                % ��ĩ������ʱ��  ��������ĩʱ��������Ϊ���ٶ�
    FinalTrip_t = FinalTrip_t - FinalImpulse_t;
    Tran = Inertial2Orbit(rv_calangel);
    Orbit_Impulse = Tran * Thrust_Impulsive(1:3, end)/norm(Thrust_Impulsive(1:3, end));    % ���ϵ����ʸ��
    [FinalImpulse_Azimuth, FinalImpulse_Elevation] = ThrustAngleCal2(Orbit_Impulse);       % ���ϵ����Ƕ�
    E_rvm = [E_rv(1:6, end);final_mass];
    [~, RVm] = ode45(@(t, RVm) J2Cal_rvm(t, RVm, Thrust_f, [FinalImpulse_Azimuth;FinalImpulse_Elevation], k), [FinalImpulse_t 0], E_rvm);     %����RV
    E_rv(:, end) = RVm(end, 1:6)';
    
    t_total(1, end) = FinalImpulse_t;
    Thrust_angle(1, end) = FinalImpulse_Azimuth;
    Thrust_angle(2, end) = FinalImpulse_Elevation;
    
    % �����������
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    pro_mass = mass;         % ��¼�����仯
    for i = 1:Thrust_num-1
        % p = cat(1, rv_c, E_rv, Thrust_f, T);  % ׷����RV ����RV �����С ����Ƕ� ת����ʱ��
        p = cat(1, rv_c, E_rv(:, i), pro_mass, Thrust_f, Thrust_Impulsive(4, i+1)-Thrust_Impulsive(4, i), k);
       
        % ���ն���Ҫ���������ת��ʱ��
        if i == Thrust_num-1
            p = cat(1, rv_c, E_rv(:, i), pro_mass, Thrust_f, FinalTrip_t, k);
        end
        
        Tran = Inertial2Orbit(rv_c);
        [Azimuth, Elevation] = ThrustAngleCal2(Tran * Thrust_Impulsive(1:3, i)/norm(Thrust_Impulsive(1:3, i)));
        x_0 = [norm(Thrust_Impulsive(1:3, i))/(Thrust_f/pro_mass/1000), Azimuth, Elevation];
        
        u_lb = [0.5*norm(Thrust_Impulsive(1:3, i))/(Thrust_f/pro_mass/1000), -1, -90];
        u_ub = [1.5*norm(Thrust_Impulsive(1:3, i))/(Thrust_f/pro_mass/1000), 360, 90];
        [output, ~, ~] = fmincon(@(x) CostFun(x, p), x_0, A, b, Aeq, beq, u_lb, u_ub);
        t_total(1, i) = output(1);
        pro_mass = pro_mass + k*output(1)
        Thrust_angle(1, i) = output(2);
        Thrust_angle(2, i) = output(3);
        [~,~,new_rv] = CostFun(output, p);
        rv_c = new_rv;
    end
    
    % ���һ����������
    u_lb = [0.5*t_total(1, end), -1, -90];
    u_ub = [1.5*t_total(1, end), 360, 90];
    x_0 = [t_total(1, end), FinalImpulse_Azimuth, FinalImpulse_Elevation];
    nonlcon = []
%     options.OptimalityTolerance=1e-12
%     options.FunctionTolerance  =1e-10
%     options.ConstraintTolerance = 1e-10
%     [output, value, ~] = fmincon(@(x) CostFun4(x, rv_c' , RVEnd', Thrust_f, x_0(1), pro_mass, k), x_0, A, b, Aeq, beq, u_lb, u_ub,nonlcon, options);
    [output, value, ~] = fmincon(@(x) CostFun4(x, rv_c' , RVEnd', Thrust_f, x_0(1), pro_mass, k), x_0, A, b, Aeq, beq, u_lb, u_ub);
%     options.PopulationSize = 30
%     [output, value, ~] = ga(@(x) CostFun4(x, rv_c' , RVEnd', Thrust_f, x_0(1), pro_mass, k),3, A, b, Aeq, beq, u_lb, u_ub,nonlcon, options);
    
    t_total(end) = output(1);
    pro_mass = pro_mass + k*output(1)
    %[output, value, ~] = fmincon(@(x) CostFun3(x, rv_c' , RVEnd', Thrust_f, x_0(1), pro_mass, k), x_0(2:3), A, b, Aeq, beq, u_lb(2:3), u_ub(2:3));
    Thrust_angle(1,end) = output(2);
    Thrust_angle(2,end) = output(3);
end

function J = CostFun4(x, rv, RV, Thrust_f, t0, m, k)
t = x(1) - t0;
newrv = OrbitPrediction(rv', -t, 1, [1 0], 'RK4');
newrvm = [newrv', m];
[~,rv2] = ode45(@(t, RV) J2Cal_rvm(t, RV, Thrust_f, [x(2);x(3)], k), [0 x(1)], newrvm');
finalrv = rv2(end, 1:6);
J = norm(RV - finalrv);
end

