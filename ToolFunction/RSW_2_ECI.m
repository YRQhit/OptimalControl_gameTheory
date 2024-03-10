%%%%%%RSW坐标系to地心惯性系转换矩阵%%%%%%%%%%%%%%%%%%%%%%

function T_RSW_ECI=RSW_2_ECI(coe)
d_2_r = pi/180;
incl = coe(3) * d_2_r;
RAAN = coe(4) * d_2_r;
omegaa = coe(5) * d_2_r;
TA = coe(6) * d_2_r;
u=omegaa+TA;%纬度幅角
T1=[cos(-RAAN)  sin(-RAAN) 0;
    -sin(-RAAN) cos(-RAAN) 0;
    0          0           1;];
T2=[1 0 0;
    0 cos(-incl)  sin(-incl);
    0 -sin(-incl) cos(-incl)];
T3=[cos(-u)  sin(-u) 0;
    -sin(-u) cos(-u) 0;
    0        0       1];
T_RSW_ECI=T1*T2*T3;
end