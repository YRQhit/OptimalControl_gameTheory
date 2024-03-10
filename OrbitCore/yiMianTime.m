% 异面交会时间窗口计算
% 输入：coe_t - 目标星六根数    coe_c - 追踪星六根数
% 输出：T1 - 目标星与追踪星平面降交时间
%       x1 - 降交对应的位置
%       T2 - 目标星与追踪星平面升交时间
%       x2 - 升交对应的位置
function [T1,x1,T2,x2] = yiMianTime(coe_t, coe_c)
global GM_Earth deg2rad rad2deg;
e = coe_t(2); a = coe_t(1);
Tperiod = 2 * pi * sqrt(a^3 / GM_Earth);
[r_t,v_t] = Orbit_Element_2_State_rv(coe_t , GM_Earth);
[r_c,v_c] = Orbit_Element_2_State_rv(coe_c , GM_Earth);
temp = cross(r_c,v_c);
hc = temp / norm(temp);
temp = cross(r_t,v_t);
ht = temp / norm(temp);

n = cross(ht, hc);              % 轨道面交线
if n == 0
    T1 = 0; T2 = 0;
    return;
end
n = n / norm(n);
r = r_t / norm(r_t);

q = coe_t(6);
q1 = acos(dot(r,n)) * rad2deg;
q2 = 180 - q1;

t0 = true2Mean(e, q) * deg2rad * Tperiod / 2 / pi;
h = cross(r,n) / norm(cross(r,n));
if norm(h - ht) > eps
    q2 = q2 + q;
    t2 = AmendDeg(true2Mean(e, q2), '0 - 360') * deg2rad * Tperiod / 2 / pi;
    T2 = t2 - t0;
    
    t3 = AmendDeg(true2Mean(e, q2 + 180),'0 - 360') * deg2rad * Tperiod / 2 / pi;
    if t3 - t2 < 0
        T1 = T2 + Tperiod - abs(t3 - t2);
    else
        T1 = T2 + t3 - t2;
    end
else
    q1 = q1 + q;
    t1 = AmendDeg(true2Mean(e, q1),'0 - 360') * deg2rad * Tperiod / 2 / pi;
    T1 = t1 - t0;
    
    t3 = AmendDeg(true2Mean(e, q1 + 180), '0 - 360') * deg2rad * Tperiod / 2 / pi;
    if t3 - t1 < 0 
        T2 = T1 + Tperiod - abs(t3 - t1);
    else
        T2 = T1 + t3 - t1;
    end
end

coe = twoBodyOrbit(coe_t,T1);
[x1,~] = Orbit_Element_2_State_rv(coe , GM_Earth);

coe = twoBodyOrbit(coe_t,T2);
[x2,~] = Orbit_Element_2_State_rv(coe , GM_Earth);
end

