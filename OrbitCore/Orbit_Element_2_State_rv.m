%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%----------------------由轨道根数求位置速度矢量-----------------------------%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%---------coe（classical orbit elements）-------------%
%a：轨道半长轴
%e：轨道偏心率
%incl：轨道倾角(单位/deg)
%RAAN：升交点赤经(单位/deg)
%omegaa：近地点幅角(单位/deg)
%TA：真近点角(单位/deg)
function [r , v] = Orbit_Element_2_State_rv(coe , muu)
if nargin == 1
    muu = 3.986004415e+05;
end

d_2_r = pi/180;
a = coe(1);
e = coe(2);
incl = coe(3) * d_2_r;
RAAN = coe(4) * d_2_r;
omegaa = coe(5) * d_2_r;
TA = coe(6) * d_2_r;

h = sqrt(a * muu * (1 - e^2));

rp = (h^2/muu) * (1/(1 + e * cos(TA))) * (cos(TA) * [1;0;0] + sin(TA) * [0;1;0]);
vp = (muu/h) * (-sin(TA) * [1;0;0] + (e + cos(TA)) * [0;1;0]);

R3_RAAN = [cos(RAAN)   sin(RAAN)   0;
           -sin(RAAN)  cos(RAAN)   0
           0           0           1];
    
R1_incl = [  1   0           0
             0   cos(incl)   sin(incl)
             0   -sin(incl)  cos(incl)];
    
R3_omegaa = [cos(omegaa)   sin(omegaa)   0;
             -sin(omegaa)  cos(omegaa)   0
             0           0               1];
         
Q_px = R3_RAAN' * R1_incl' * R3_omegaa';

r = Q_px * rp;
v = Q_px * vp;

end