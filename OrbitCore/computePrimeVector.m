% 计算主矢量
% 输入：chaseData - 轨道数据     T - 总时间    t0 - 初始时刻
%         step - 步长         r0 - 协态初值
function [r1,r2] = computePrimeVector(chaseData, T, t0, step, r0)
n = size(chaseData, 2);
r1 = zeros(3,n);   r2 = r1;

if norm(r0(:,1)) == 0
    t0 = T;
    T = -T;
    step = -step;
    temp = r0(:,1);
    r0(:,1) = r0(:,2);
    r0(:,2) = temp;
end

[N, ~] = computeN(chaseData(:,end),t0 + T);
% [~, N_1] = computeN(chaseData(:,1),t0);
[N_1, ~] = computeN(chaseData(:,1),t0);
N_1 = inv(N_1);
Q = N*N_1;
A = Q(1:3,1:3);   B = Q(1:3,4:6);
r = inv(B)*(r0(:,2) - A*r0(:,1));

for i = 1:n
    if i == 1
        r1(:,1) = r0(:,1);      r2(:,1) = r;
        continue;
    elseif i~=n
        [N, ~] = computeN(chaseData(:,i),t0 + (i-1) * step);
    else
        [N, ~] = computeN(chaseData(:,i),t0 + T);
    end
    Q = N*N_1;
    A = Q(1:3,1:3);   B = Q(1:3,4:6);
    C = Q(4:6,1:3);   D = Q(4:6,4:6);
    
    r1(:,i) = A * r0(:,1) + B * r;
    r2(:,i) = C * r0(:,1) + D * r;
end

if T < 0
    r1 = fliplr(r1);
    r2 = fliplr(r2);
end
    
end

function [N, N_1] = computeN(chasePosVel,t)
global deg2rad GM_Earth;
rc = chasePosVel(1:3);       vc = chasePosVel(4:6);
coe = State_rv_2_Orbit_Element(rc, vc);
a = coe(1);        e = coe(2);       q = coe(6)*deg2rad;
H = cross(rc,vc);            h = norm(H);   
r = norm(rc);                p = a*(1+e)*(1-e);

f = -(p/h)*(p+r)*r*cos(q);
if e~=1
    g = ((p/h)*(p+r)*r*sin(q) - 3*e*p*t) / (1-e^2);
else
    g = 2/5*(3*p*t - (r^3/h)*sin(q));
end
F1 = r*cos(q);              F2 = r*sin(q);           F3 = -(h/p)*cos(q);
F4 = (h/p)*(e+cos(q));      F5 = GM_Earth/r^3;       F6 = 3*t;
F7 = 3*GM_Earth*t/r^3;      F8 = F3 + g*F5;          F9 = F4 + f*F5;
N = [F1*rc-g*vc    F2*rc-f*vc   2*rc-F6*vc   vc     F1*H    F2*H;
     F8*rc-F1*vc   F9*rc-F2*vc  F7*rc-vc   -F5*rc   F3*H    F4*H];

Q = cross(rc,H)';             w = cross(vc,H)';
b1 = h+F5*(F1*f-F2*g);       b2 = F6*h+F3*f-F4*g;
b3 = F6*h+2*(F3*f-F4*g);     b4 = F1*f-F2*g;
N_1=[F2*F5*Q-F4*w     2*F4*Q-F2*w;
     -F1*F5*Q+F3*w    -2*F3*Q+F1*w;
      h*w             -h*Q;
      -b1*Q+b2*w      -b3*Q+b4*w;
      F4*H'            -F2*H';
      -F3*H'           F1*H'] / h^3;
end

