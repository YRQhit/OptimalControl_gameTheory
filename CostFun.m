%%  ���ۺ���
% x = [����ʱ�� ����Ƕ�2*1]
% p = cat(1, rv_c, E_rv, m, Thrust_f, T);  % ׷����RV ����RV �����С ����Ƕ� ת����ʱ��

function [minJ, err, rv] = CostFun(x, p)
    rv_c = p(1:6, :);
    E_rv = p(7:12, :);
    m = p(13);
    Thrust_f = p(14);
    T = p(15);
    k = p(16);
    rvm = [rv_c;m];
    [~, RV] = ode45(@(t, RV) J2Cal_rvm(t, RV, Thrust_f, [x(2);x(3)], k), [0 x(1)], rvm);
%     [~, RV] = ode45(@(t, RV) J2Cal(t, RV, Thrust_f, [x(2);x(3)]), [0 x(1)], rv_c);

    rv = RV(end, 1:6)';
    rv =  OrbitPrediction(rv, T-x(1), 60, [1 0], 'RK4');      % ��������
%     rv = J2OrbitRV(rv, T-x(1));                                % J2����
%     rv = OrbitPrediction(rv, T-x(1), 60, [1 1], 'RK7');
    err = E_rv - rv;
    minJ = norm(E_rv - rv);
end