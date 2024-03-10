function s = stumpS(z)
% stumpS  Vectorize Stumpff function S(z)
%
%   s = stumpS(z) compute Stumpff function S(z). The stumpS function
%   operates element-wise on arrays.  
%
%   See also: stumpC.

% Copyright 2018 Zhu Qiangjun.
% $Revision: 1.0.0.1 $  $Date: 2018/11/07 15:39:00 $


s = zeros(size(z));

ze = z(z > 0);
zh = z(z < 0); 

% z > 0, ellipse 
s(z > 0) = (sqrt(ze) - sin(sqrt(ze))) ./ (sqrt(ze)).^3;

% z == 0, parabola
s(z == 0) = 1/6;

% z < 0, hyperbola
s(z < 0) = (sinh(sqrt(-zh)) - sqrt(-zh)) ./ (sqrt(-zh)).^3;

return
