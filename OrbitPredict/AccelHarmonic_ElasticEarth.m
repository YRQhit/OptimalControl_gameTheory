% ���������
% Mjd - ������(UTC)         r - ���Ĺ���ϵ�µ�����         
% deg - �������㶯����      E - ����ϵ���ع�ϵ��ת������
% ���������
% a - �������㶯������������ٶ�
function a = AccelHarmonic_ElasticEarth(r,deg)
global GM_Earth r_E C S E;       
r_bf   = E * r;                                       % ������λ��ʸ��r�ɵع�ϵICRF���ع�ϵITRS
[lon, latgc, d] = CalcPolarAngles(r_bf);              % �������γ�ȡ����ľ��ȡ����ľ�

if ( lon > pi )
    lon = lon - 2*pi;
end

[pnm, dpnm] = Legendre(deg,deg,latgc); 
dUdr = 0; dUdlat = 0; dUdlon = 0;
q1 = 0;  q2 = 0;  q3 = 0;

for n=2:deg
    b1 = (-GM_Earth/d^2)*(r_E/d)^n*(n+1);
    b2 = (GM_Earth/d)  *(r_E/d)^n;
    b3 = (GM_Earth/d)  *(r_E/d)^n;
    for m=0:n
        ml =  m*lon;
        q1 = q1 + pnm(n+1,m+1)  * (S(n+1,m+1)*sin(ml) + C(n+1,m+1)*cos(ml));
        q2 = q2 + dpnm(n+1,m+1) * (S(n+1,m+1)*sin(ml) + C(n+1,m+1)*cos(ml));
        q3 = q3 + m*pnm(n+1,m+1)* (S(n+1,m+1)*cos(ml) - C(n+1,m+1)*sin(ml));
    end
  
    dUdr   = dUdr     + q1*b1;                         % U�Ե��ľ�rƫ����
    dUdlat = dUdlat   + q2*b2;                         % U�Ե��ľ��Ȧ�ƫ����
    dUdlon = dUdlon   + q3*b3;                         % U�Ե���γ�Ȧ�ƫ����
    q1 = 0;  q2 = 0;  q3 = 0;
end
x = r_bf(1);
y = r_bf(2);
z = r_bf(3);

xy2 = x^2+y^2;
xyn = sqrt(xy2);

R_sph2rec(:,1) = [x         y       z  ]'/d;
R_sph2rec(:,2) = [-x*z/xyn -y*z/xyn xyn]'/d^2;
R_sph2rec(:,3) = [-y        x       0  ]'/xy2;

a_bf =  R_sph2rec*[dUdr;dUdlat;dUdlon];
a = E'*a_bf;
end

%% ������ľ�γ��
% ���룺r_bf - �ع�ϵ�µ�����
% �����lon - ����ά��   latgc - ���ľ���    d - ���ľ� 
function [lon, latgc, d] = CalcPolarAngles(r_bf)
rhoSqr = r_bf(1) * r_bf(1) + r_bf(2) * r_bf(2); 
d = sqrt(rhoSqr + r_bf(3) * r_bf(3));

if ( (r_bf(1)==0) && (r_bf(2)==0) )
    lon = 0;
else
    lon = atan2(r_bf(2), r_bf(1));
end

if ( lon < 0 )
    lon = lon + 2*pi;
end
rho = sqrt( rhoSqr );
if ( (r_bf(3)==0) && (rho==0) )
    latgc = 0;
else
    latgc = atan2(r_bf(3), rho);
end
end

%% �������õ²�����
function [pnm, dpnm] = Legendre(n,m,fi)
sf = sin(fi);
cf = cos(fi);

pnm  	= zeros(n+1,m+1);
dpnm    = zeros(n+1,m+1);

pnm(1,1)  =  1;
dpnm(1,1) =  0;
pnm(2,2)  =  sqrt(3)*cf;
dpnm(2,2) = -sqrt(3)*sf;

for i=2:n    
    pnm(i+1,i+1)  = sqrt((2*i+1)/(2*i))*  cf*pnm(i,i);
    dpnm(i+1,i+1) = sqrt((2*i+1)/(2*i))*( cf*dpnm(i,i) - sf*pnm(i,i) );
end

for i=1:n
    pnm(i+1,i)    = sqrt(2*i+1)*sf*pnm(i,i);
    dpnm(i+1,i)   = sqrt(2*i+1)*( cf*pnm(i,i) + sf*dpnm(i,i) );
end

j=0;
k=2;
while(1)
    for i=k:n        
        pnm(i+1,j+1) =sqrt((2*i+1)/((i-j)*(i+j))) * ((sqrt(2*i-1)*sf* pnm(i,j+1))-(sqrt(((i+j-1)*(i-j-1))/(2*i-3))*pnm(i-1,j+1)));
        dpnm(i+1,j+1)=sqrt((2*i+1)/((i-j)*(i+j))) * ((sqrt(2*i-1)*sf*dpnm(i,j+1))+(sqrt(2*i-1)*cf*pnm(i,j+1))-(sqrt(((i+j-1)*(i-j-1))/(2*i-3))*dpnm(i-1,j+1)));
    end
    j = j+1;
    k = k+1;
    if (j>m)
        break
    end
end
end






