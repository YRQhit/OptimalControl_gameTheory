function SunPos = CalAstroBodyPos(JD)
%地球指向太阳的矢量
JDTDB = CalJDTDB(JD);
d2r = pi/180;
JCTDB = (JDTDB-2451545)/36525;

GM_s = 1.327122000000e+011;     %太阳引力常数，单位km^3/s^2
EcliOblJ2000 = 23.43929*d2r;    %J2000.0黄赤交角，单位rad
ECI2SCI = angle2dcm(EcliOblJ2000,0,0,'XYZ');    %地心惯性坐标系到日心黄道坐标系的转换矩阵


AU = 1.49597871*1e8;
EarthPosSCI = CalAstroBodyPosSCI(JCTDB,GM_s,1.00000011*AU,-0.00000005*AU,0.01671022,-0.00003804,0.00005*d2r,-46.94/3600*d2r,-11.26064*d2r,-18228.25/3600*d2r,102.94719*d2r,1198.28/3600*d2r,100.46435*d2r,129597740.63/3600*d2r);
SunPos = -ECI2SCI'*EarthPosSCI;
end

function AstroBodyPosSCI = CalAstroBodyPosSCI(JCTDB,GM,a_0,da_0,e_0,de_0,i_0,di_0,Omegaa_0,dOmegaa_0,hat_omegaa_0,dhat_omegaa_0,L_0,dL_0)
a = CalOrbEle(JCTDB,a_0,da_0);
e = CalOrbEle(JCTDB,e_0,de_0);
i = CalOrbEle(JCTDB,i_0,di_0);
Omegaa = CalOrbEle(JCTDB,Omegaa_0,dOmegaa_0);
hat_omegaa = CalOrbEle(JCTDB,hat_omegaa_0,dhat_omegaa_0);
L = CalOrbEle(JCTDB,L_0,dL_0);
h = sqrt(GM*a*(1-e^2));
omegaa= hat_omegaa_0-Omegaa;
M = L-hat_omegaa;
E = kepler_E(e,M);
thetaa = atan(tan(E/2)*sqrt((1+e)/(1-e)))*2;
AstroBodyPosSCI = CalPos(h,e,Omegaa,i,omegaa,thetaa,GM);
end

function OrbEle = CalOrbEle(JCTDB,OrbEle_0,dOrbEle_0)
OrbEle = OrbEle_0+dOrbEle_0*JCTDB;
end

function E = kepler_E(e,M)
err = 1e-10;
if M < pi
    E = M+e/2;
else
    E = M-e/2;
end
ratio = 1;
while abs(ratio) > err
    ratio = (E-e*sin(E)-M)/(1-e*cos(E));
    E = E-ratio;
end

end

function Pos = CalPos(h,e,Omegaa,i,omegaa,thetaa,GM)
rp = (h^2/GM)*(1/(1+e*cos(thetaa)))*(cos(thetaa)*[1;0;0]+sin(thetaa)*[0;1;0]);
R3_Omegaa = [cos(Omegaa) sin(Omegaa) 0;-sin(Omegaa) cos(Omegaa) 0;0 0 1];
R1_i = [1 0 0;0 cos(i) sin(i);0 -sin(i) cos(i)];
R3_omegaa = [cos(omegaa) sin(omegaa) 0;-sin(omegaa) cos(omegaa) 0;0 0 1];
Q_pX = R3_Omegaa'*R1_i'*R3_omegaa';
r = Q_pX*rp;
r = reshape(r,3,1);
Pos = r;
end

function JDTDB = CalJDTDB(JD)
DAT = 36;    %TAI-UTC，单位s
JDTT = JD+(DAT+32.184)/24/3600;
JDTDB = JDTT+(0.001657*sin(6.24+0.017202*(JDTT-2451545.0)))/24/3600;
end