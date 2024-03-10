% 共面四脉冲转移
% 输入：r0 - 初始轨道高度      r2 - 目标轨道高度  
%       q - 相角差             T - 总时间
% 输出：r1 - 过渡轨道高度      T1 - 过渡轨道漂飞时间
%       Tw1,Tw2 - 转移时间     Dv - 总速度增量
function control = fourPulseTransfer(r0, r2, q, T)
global GM_Earth;
Tperiod = 2*pi*sqrt(r0^3/GM_Earth);
kmin = floor((T - Tperiod) / Tperiod) -1;
kmax = ceil(T / Tperiod);

if kmin < 0
    kmin = 0;
end
if kmax < 0
    kmax = 0;
end

    function y = paramSolve(k)
        f = @(r1)(2 * pi - 2 * k * pi - q + ...
            (T - pi * sqrt((r0 + r1)^3 / 8 / GM_Earth) - ...
            pi * sqrt((r1 + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1^3));
        y =  fsolve(f,r0);
    end

    function y = newtonSolve(k)
    r1 = r0; r1pre = 0;j = 0;
    while(abs(r1 - r1pre) > eps)
        r1pre = r1;
        f = (2 * pi - 2 * k * pi - q + ...
            (T - pi * sqrt((r0 + r1)^3 / 8 / GM_Earth) - ...
            pi * sqrt((r1 + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1^3));
        g = (3*GM_Earth*(pi*((r0 + r1)^3/(8*GM_Earth))^(1/2) - T + pi*((r1 + r2)^3/(8*GM_Earth))^(1/2)))...
            /(2*r1^4*(GM_Earth/r1^3)^(1/2)) - ((3*pi*(r0 + r1)^2)/(16*GM_Earth*((r0 + r1)^3/(8*GM_Earth))^(1/2))...
            + (3*pi*(r1 + r2)^2)/(16*GM_Earth*((r1 + r2)^3/(8*GM_Earth))^(1/2)))*(GM_Earth/r1^3)^(1/2);
        r1 = r1 - f / g;
        j = j + 1;
        if j == 50
            break;
        end
    end
    y = r1;
    end

    function y = dichotomySolve(k)
        r1 = 6600:20:8000;
        f = (2 * pi - 2 * k * pi - q + ...
            (T - pi * sqrt((r0 + r1).^3 / 8 / GM_Earth) - ...
            pi * sqrt((r1 + r2).^3 / 8 / GM_Earth)) .* sqrt(GM_Earth ./ r1.^3));
        if min(f) > 0 || max(f) < 0
            y = nan;
            return;
        end
        
        [~,indexA] = min(abs(f));
        if indexA == 1
            indexB = 2;
        elseif indexA == length(f)
            indexB = indexA - 1;
        else
             if f(indexA - 1) * f(indexA) < 0
                indexB = indexA - 1;
             else
                indexB = indexA + 1;
             end
        end
        
        A = r1(indexA); B = r1(indexB);
        y = (A + B) / 2;
        for i = 1:4
            temp = (2 * pi - 2 * k * pi - q + ...
                   (T - pi * sqrt((r0 + y)^3 / 8 / GM_Earth) - ...
                    pi * sqrt((y + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / y^3));
            if temp * A < 0
                y = (y + A) / 2;
            else
                y = (y + B) / 2;
            end
        end
    end

n = kmax - kmin + 1;
r1temp = zeros(1,n); Dvtemp = zeros(5,n);
Twtemp = zeros(2,n);
for k = kmin:kmax
    r1temp(k - kmin + 1) = newtonSolve(k);
%     r1temp(k - kmin + 1) = dichotomySolve(k);
%       r1temp(k - kmin + 1) = paramSolve(k);

    temp = (2 * pi - 2 * k * pi - q + (T - pi * sqrt((r0 + r1temp(k - kmin + 1))^3 / 8 / GM_Earth) - ...
            pi * sqrt((r1temp(k - kmin + 1) + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1temp(k - kmin + 1)^3));
    if ~isreal(temp) || isnan(temp) || r1temp(k - kmin + 1) < 6400
        Dvtemp(5,k - kmin + 1) = 1e5;
        continue;
    end
    
    [Dvtemp(1,k - kmin + 1), Dvtemp(2,k - kmin + 1), Twtemp(1,k - kmin + 1)] = ...
            Hm_transfer(r0, r1temp(k - kmin + 1));
    [Dvtemp(3,k - kmin + 1), Dvtemp(4,k - kmin + 1), Twtemp(2,k - kmin + 1)] = ...
            Hm_transfer(r1temp(k - kmin + 1), r2);
    Dvtemp(5,k - kmin + 1) = norm(Dvtemp(1:4,k - kmin + 1),1);
    
    if T - Twtemp(1,k - kmin + 1) - Twtemp(2,k - kmin + 1) < 0
       Dvtemp(5,k - kmin + 1) = 1e5;
    end
end
[control.Dv,index] = min(Dvtemp(5,:)); control.r1 = r1temp(index);
control.dv1 = Dvtemp(1,index)*1000; control.dv2 = Dvtemp(2,index)*1000;
control.dv3 = Dvtemp(3,index)*1000; control.dv4 = Dvtemp(4,index)*1000;
control.Tw1 = Twtemp(1,index); control.Tw2 = Twtemp(2,index);
control.T1 = T - control.Tw1 - control.Tw2;
end

