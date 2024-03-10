% 共面圆轨道交会
% 输入：r0 - 初始轨道高度          r2 - 目标轨道高度 
%       q - 相角差       T - 总时间      n - 脉冲个数
% 例：control = circleRendezvous(6885, 6900, 0.1, 15000, 3);
function control = circleRendezvous(r0, r2, q, T,n)
if nargin == 4
    n = 3;
end

if n == 3
    control = threePulseRendezvous(r0, r2, q, T);
elseif n == 4
    control = fourPulseRendezvous(r0, r2, q, T);
else
    err('脉冲个数有误');
end

end

%% 共面三脉冲交会
% 输出：t1,t2,t3 - 脉冲前的漂飞时间      dv1,dv2,dv3 - 三次脉冲
%       Dv - 总速度增量
function control = threePulseRendezvous(r0, r2, q, T)
global GM_Earth;
w0 = sqrt(GM_Earth/r0^3);  w2 = sqrt(GM_Earth/r2^3);
[~,~,control.t3] = Hm_transfer(r0,r2);
Tperiod = 2*pi*sqrt(r0^3/GM_Earth);
Kmax = ceil(T / Tperiod);
param = zeros(Kmax,1);
t1 = zeros(Kmax,1);    t2 = zeros(Kmax,1);
T2 = t2;
for k = 1:Kmax
    t1(k) = (w2*T+q-2*k*pi-pi)/w0;
    t2(k) = T - t1(k) - control.t3;
    if t1(k) < 0 || t2(k) < 0
        param(k) = realmax;
        continue;
    end
    
    T2(k) = t2(k) / k;
    param(k) = abs(T2(k) - Tperiod);
end
[~,index] = min(param);
a = (T2(index)^2*GM_Earth/4/pi/pi)^(1/3);
e = (a - r0)/a;

control.t1 = t1(index);     control.t2 = t2(index);
[control.dv1,~,~] = Hm_transfer(r0,2*a-r0);
control.dv1 = 1000 * control.dv1;
v1 = sqrt(GM_Earth*(1+e)/r0);

a = (r0+r2)/2;
e = (a - r0)/a;
v2 = sqrt(GM_Earth*(1+e)/r0);
control.dv2 = 1000*(v2 - v1);

v3 = sqrt(GM_Earth*(1-e)/r2);
v4 = sqrt(GM_Earth/r2);
control.dv3 = 1000*(v4 - v3);
control.Dv = abs(control.dv1) + abs(control.dv2) + abs(control.dv3);
control.Dv = control.Dv / 1000;
end

%% 共面四脉冲交会
% 输出：Tw1，Tw2，T1- 各段时间          dv1,dv2,dv3,dv4 - 四次脉冲 
%       Dv - 总速度增量                 r1 - 过渡轨道高度
function control = fourPulseRendezvous(r0, r2, q, T)
global GM_Earth;
f =  @(r1)(2 * pi - T * sqrt(GM_Earth / r2^3) - q + ...
    (T - pi * sqrt((r0 + r1)^3 / 8 / GM_Earth) - ...
    pi * sqrt((r1 + r2)^3 / 8 / GM_Earth)) * sqrt(GM_Earth / r1^3));
control.r1 = fsolve(f,r0);
[control.dv1, control.dv2, control.Tw1] = Hm_transfer(r0, control.r1);
[control.dv3, control.dv4, control.Tw2] = Hm_transfer(control.r1, r2);
control.Dv = norm(control.dv1) + norm(control.dv2) + norm(control.dv3) + norm(control.dv4);
control.dv1 = 1000*control.dv1;control.dv2 = 1000*control.dv2;
control.dv3 = 1000*control.dv3;control.dv4 = 1000*control.dv4;
control.Tw = T - control.Tw1 - control.Tw2;
end
