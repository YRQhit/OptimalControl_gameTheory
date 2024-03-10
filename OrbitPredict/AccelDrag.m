% 计算大气摄动加速度
% 输入参数：
% PosVel - 卫星位置速度       dens - 大气密度（不给定时采用指数模型计算）
% 输出参数：
% a - 大气摄动产生的三轴加速度
function a = AccelDrag(PosVel,dens)
global CD s_m;
r = sqrt(PosVel(1)^2 + PosVel(2)^2 + PosVel(3)^2);
if nargin == 1
    dens = ComputeDenstiy(r);
end
w = 0.7292e-4;
v_rel = [PosVel(4) + w * PosVel(2);
         PosVel(5) - w * PosVel(1);
         PosVel(6)];
a = -0.5 * CD * s_m * 1e3 * dens * norm(v_rel) * v_rel;
end

%% 计算大气密度（指数模型）
% 输入：r - 卫星地心距   输出：den - 大气密度
function den = ComputeDenstiy(r)
p0 = 3.6e-10;  H0 = 37.4;
r0 = 6408.4;   
H = H0 + 0.12 * (r - r0);                 
den = p0 * exp(-(r - r0) / H);
end

