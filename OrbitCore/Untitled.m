% 最省燃料法
x1 = [6885;0;0;0;7.60881;0];
x2 = [6664.89;1785.85;0;-1.96716;7.34156;0];
global T0 q T1 w1 r1 r2;
r1 = norm(x1(1:3)); r2 = norm(x2(1:3));
r3 = (r1 + r2) / 2;
w1 = sqrt(GM_Earth / r1^3);
w2 = sqrt(GM_Earth / r2^3);
w3 = sqrt(GM_Earth / r3^3);

q = acos(dot(x1(1:3), x2(1:3)) / norm(x1(1:3)) / norm(x2(1:3)));
T0 = 2*pi / w3; T1 = 2*pi / w1;
% t1 = (-w2*(T - N*T0) + (q + pi))/(w1 - w2);
% t2 = (w1*(T - N*T0) - (q + pi))/(w1 - w2);
% 
% T2 = zeros(1,10);
% T1 = T2;
% for N = 0.5:0.5:5.5
%     T2(N/0.5) = (q + pi)/w2 + N*T0
%     T1(N/0.5) = (q + pi)/w1 + N*T0
% end
 
%% 任意调相
t = 6000:500:25000;  Jmin = 86400;
tic;
N = length(t);fuel = zeros(1,N);
for m = 1:N
    [k,N,T2,Dv] = tiaoSolve(t(m));
    fuel(m) = fuelCost(Dv,1500,285);
end
toc;

function [k,N,T2,Dv] = tiaoSolve(t)
global T0 q T1 w1 r1 r2 GM_Earth;
Jmin = 86400;T2optimal = 0;
for i = 1:10
    for j = 1:10
       T2 = (t - 0.5 * T0 - (q + 2*i*pi - pi) / w1) / j;
       J = abs(T2 - T1);
       if J < Jmin
          Jmin = J;
          k = i;N = j;
          T2optimal = T2;
       end
    end
end
rp = 2*(T2optimal^2*GM_Earth/4/pi/pi)^(1/3) - r1;
e = (rp - r1)/(rp + r1);
h = sqrt(r1*(1 + e)*GM_Earth);
deltv = h / r1 - w1 * r1
[dv1, dv2, ~] = Hm_transfer(r1, r2);
Dv = 2 * abs(deltv) + abs(dv1) + abs(dv2);
end




    
    