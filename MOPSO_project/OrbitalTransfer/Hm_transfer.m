% 用以计算二体情形霍曼转移的deltv1（单位km/s) 和 Tw（单位s)
% r1为初始轨道高度，r2为目标轨道高度(单位km)
function [deltv1,deltv2,Tw]= Hm_transfer(r1,r2)
global GM_Earth;
deltv1 = sqrt(GM_Earth / r1) * (sqrt((2*r2) / (r1 + r2)) - 1);
deltv2 = sqrt(GM_Earth / r2) * (1 - sqrt((2*r1) / (r1 + r2)));
r_avg = (r1+r2)/2;
Tw = pi*sqrt(r_avg^3/GM_Earth);
end