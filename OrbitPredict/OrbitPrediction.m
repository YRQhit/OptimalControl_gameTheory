% 轨道递推（考虑非球形摄动以及大气摄动） 输入：x0 - 初始的位置速度   time - 递推时间
%       h - 递推步长          integrator - 积分器类型（RK4/RK7） model - 是否考虑摄动项（1/0）
%       [非球形 大气] statTime - 初始时刻（给定该值后采用高精度递推）
% 输出：x - 最终的位置速度    xt - 递推产生的中间数据
function [x,xt] = OrbitPrediction(x0,time,h,model,intergrator,startTime)
constants;
global GM_Earth E deg orbitModel;
x = x0;  xt = x0; 
xk = zeros(1,4);    k4 = zeros(4,6);   k7 = zeros(13,6);
M_T = 1 / 86400;    Mjd = 0;              
h = h * sign(time);                                  % 由递推方向决定步长   

if rem(time,h) == 0                                  % 计算最后一步步长（非等间隔）
    finalStep = h;
    num = floor(time / h);
else
    finalStep = rem(time,h);
    num = floor(time / h) + 1;
end

if nargin == 3 
    model = [1 1];                        % 默认考虑非球形即大气摄动
end

if nargin <= 4
    intergrator = 'RK4';
end

if nargin == 6 
    year = startTime(1);    mon = startTime(2);    day = startTime(3);
    hour = startTime(4);    min = startTime(5);    sec = startTime(6);    
    Mjd0 = Mjday(year, mon, day, hour, min, sec);  Mjd = Mjd0;
    E = ICRS2ITRS(Mjd);    
    dens = ComputeDenstiy_HPOP(x0, startTime, 0);
end

if strcmp(intergrator,'RK4')
    param_k = [0 1/2 1/2 1];
    param_t = [1 2 2 1];
elseif strcmp(intergrator,'RK7')
    param_t = [0.0, 2.0 / 27.0, 1.0 / 9.0, 1.0 / 6.0, 5.0 / 12.0, 1.0 / 2.0, 5.0 / 6.0, 1.0 / 6.0, 2.0 / 3.0, 1.0 / 3.0, 1.0, 0.0, 1.0];
    param_c = [0.0, 0.0, 0.0, 0.0, 0.0, 34.0 / 105.0, 9.0 / 35.0, 9.0 / 35.0, 9.0 / 280.0, 9.0 / 280.0, 0.0, 41.0 / 840.0, 41.0 / 840.0];

    param_k(1,:) = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(2,:) = [2.0 / 27.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(3,:) = [1.0 / 36.0, 1.0 / 12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(4,:) = [1.0 / 24.0, 0.0, 1.0 / 8.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(5,:) = [5.0 / 12.0, 0.0, -25.0 / 16.0, 25.0 / 16.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(6,:) = [1.0 / 20.0, 0.0, 0.0, 1.0 / 4.0, 1.0 / 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(7,:) = [-25.0 / 108.0, 0.0, 0.0, 125.0 / 108.0, -65.0 / 27.0, 125.0 / 54.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(8,:) = [31.0 / 300.0, 0.0, 0.0, 0.0, 61.0 / 225.0, -2.0 / 9.0, 13.0 / 900.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    param_k(9,:) = [2.0, 0.0, 0.0, -53.0 / 6.0, 704.0 / 45.0, -107.0 / 9.0, 67.0 / 90.0, 3.0, 0.0, 0.0, 0.0, 0.0];
    param_k(10,:) = [-91.0 / 108, 0.0, 0.0, 23.0 / 108.0, -976.0 / 135.0, 311.0 / 54.0, -19.0 / 60.0, 17.0 / 6.0, -1.0 / 12.0, 0.0, 0.0, 0.0];
    param_k(11,:) = [ 2383.0 / 4100.0, 0.0, 0.0, -341.0 / 164.0, 4496.0 / 1025.0, -301.0 / 82.0, 2133.0 / 4100.0, 45.0 / 82.0, 45.0 / 164.0, 18.0 / 41.0, 0.0, 0.0];
    param_k(12,:) = [3.0 / 205.0, 0.0, 0.0, 0.0, 0.0, -6.0 / 41.0, -3.0 / 205.0, -3.0 / 41.0, 3.0 / 41.0, 6.0 / 41.0, 0.0, 0.0];
    param_k(13,:) = [-1777.0 / 4100.0, 0.0, 0.0, -341.0 / 164.0, 4496.0 / 1025.0, -289.0 / 82.0, 2193.0 / 4100.0, 51.0 / 82.0, 33.0 / 164.0, 12.0 / 41.0, 0.0, 1.0];
end

if strcmp(intergrator,'RK4')
    for i = 1 : num
        if i == num
            h = finalStep;
        end
        for j = 1:4
            for k = 1:6
                xk(k) = 0;
                if j > 1
                    xk(k) = param_k(j) * h * k4(j - 1,k);
                end
            end
            k4(j,1) = x(4) + xk(4);   k4(j,2) = x(5) + xk(5);  k4(j,3) = x(6) + xk(6);
        
            if nargin <= 5                                                                          % 低精度递推
               a = Accel(x,xk,model);
               
            elseif nargin == 6 && strcmp(orbitModel,'LPOP')
               a = Accel(x,xk,model,dens);
               
            elseif nargin == 6 && strcmp(orbitModel,'HPOP')                                                                    % 高精度递推
                E = ICRS2ITRS(Mjd + M_T * h * param_k(j));
                % 二体引力
                r = ((x(1) + xk(1))^2 + (x(2) + xk(2))^2 + (x(3) + xk(3))^2)^1.5;
                a = -GM_Earth / r * [x(1) + xk(1);x(2) + xk(2);x(3) + xk(3)];
        
                % 非球形摄动
                if model(1) == 1
                    R = [x(1) + xk(1);x(2) + xk(2);x(3) + xk(3)];
                    a = a + AccelHarmonic_ElasticEarth(R,deg);
                end
        
                % 大气摄动
                if model(2) == 1
                    dens = ComputeDenstiy_HPOP(x + xk' , startTime , Mjd - Mjd0 + M_T * h * param_k(j));
                    a = a + AccelDrag(x + xk',dens);
                end
                
                % 三体摄动
                if length(model) == 3 && model(3) == 1
                    a = a + AccelThird(x(1:3) + xk(1:3)' , Mjd + M_T * h * param_k(j));
                end
            end
             k4(j,4) = a(1);  k4(j,5) = a(2);  k4(j,6) = a(3);
        end
        Mjd = Mjd + M_T * h;
        x = x + h * k4' * param_t' / 6;    xt = [xt x];
    end
elseif strcmp(intergrator,'RK7')
    for i = 1 : num
        if i == num
            h = finalStep;
        end
        for j = 1:13
                xk = [0 0 0 0 0 0];
                if j > 1
                    for n = 1:j-1
                        xk = xk + param_k(j,n)*k7(n,:);
                    end 
                end
                k7(j,1) = h * (x(4) + xk(4)); k7(j,2) = h * (x(5) + xk(5));  k7(j,3) = h * (x(6) + xk(6));
                
                if nargin == 5
                    a = h * Accel(x,xk,model);

                elseif nargin == 6 && strcmp(orbitModel,'LPOP')
                    a = h * Accel(x,xk,model,dens);
                     
                elseif nargin == 6 && strcmp(orbitModel,'HPOP')  
                    E = ICRS2ITRS(Mjd + M_T * h * param_t(j));
                    % 二体引力
                    r = ((x(1) + xk(1))^2 + (x(2) + xk(2))^2 + (x(3) + xk(3))^2)^1.5;
                    a = -h * GM_Earth / r * [x(1) + xk(1);x(2) + xk(2);x(3) + xk(3)];
        
                    % 非球形摄动
                    if model(1) == 1
                        R = [x(1) + xk(1);x(2) + xk(2);x(3) + xk(3)];
                        a = a + h * AccelHarmonic_ElasticEarth(R,deg);
                    end
        
                    % 大气摄动
                    if model(2) == 1
                        dens = ComputeDenstiy_HPOP(x + xk',startTime,Mjd - Mjd0);
                        a = a + h * AccelDrag(x + xk',dens);
                    end
                    
                    % 三体摄动
                    if length(model) == 3 && model(3) == 1
                        a = a + h * AccelThird(x(1:3) + xk(1:3)' , Mjd + M_T * h * param_t(j));
                    end
                end
                
                k7(j,4) = a(1);  k7(j,5) = a(2);  k7(j,6) = a(3);  
        end
        Mjd = Mjd + M_T * h;
        x = x + k7'*param_c';    xt = [xt x];
    end
end
end

function a = Accel(x,xk,model,dens)
global GM_Earth r_E J2;
r = sqrt((x(1) + xk(1))^2 + (x(2) + xk(2))^2 + (x(3) + xk(3))^2);
if model(1) == 0
    a = -GM_Earth / r^3 * [x(1) + xk(1);x(2) + xk(2);x(3) + xk(3)];
elseif model(1) == 1
    a(1) = -GM_Earth * (x(1) + xk(1))/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (1 - 5 * (x(3) + xk(3))^2 / r^2));
    a(2) = a(1) * (x(2) + xk(2)) / (x(1) + xk(1));
    a(3) = -GM_Earth * (x(3) + xk(3))/ r^3 * (1 + 1.5 * J2 * (r_E / r)^2 * (3 - 5 * (x(3) + xk(3))^2 / r^2));
end
            
if model(2) == 1
    if nargin == 3
        a = a' + AccelDrag(x + xk');
    else
        a = a' + AccelDrag(x + xk',dens);
    end
end
end
















