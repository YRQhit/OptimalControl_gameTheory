%--------------------------------------------------------------------------
%
% Geodetic: geodetic coordinates (Longitude [rad], latitude [rad],
%           altitude [m]) from given position vector (r [m])
%
% 
%--------------------------------------------------------------------------
function [lon, lat, h] = Geodetic(r)
global r_E
f = 1 / 298.257223563;

epsRequ = eps*r_E*1000;      % Convergence criterion
e2      = f*(2-f);        % Square of eccentricity

X = r(1) * 1000;                 % Cartesian coordinates
Y = r(2) * 1000;
Z = r(3) * 1000;
rho2 = X*X + Y*Y;         % Square of distance from z-axis

% Check validity of input data
if (norm(r)==0)
    disp ( ' invalid input in Geodetic constructor\n' );
    lon = 0;
    lat = 0;
    h   = -r_E;
    return
end

% Iteration 
dZ = e2*Z;

while(1)
    ZdZ    = Z + dZ;
    Nh     = sqrt ( rho2 + ZdZ*ZdZ ); 
    SinPhi = ZdZ / Nh;    % Sine of geodetic latitude
    N      = r_E*1000 / sqrt(1-e2*SinPhi*SinPhi);
    dZ_new = N*e2*SinPhi;
    if (abs(dZ-dZ_new) < epsRequ)
        break
    end
    dZ = dZ_new;
end

% Longitude, latitude, altitude
lon = atan2(Y, X) * 180 / 3.14159;
lat = atan2(ZdZ, sqrt(rho2)) * 180 / 3.14159;
h   = (Nh - N);

