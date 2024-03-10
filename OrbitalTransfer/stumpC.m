function c = stumpC(z)
% stumpC  Vectorize Stumpff function C(z)
%
%   c = stumpC(z) compute Stumpff function C(z). The stumpC function
%   operates element-wise on arrays.  
%
%   See also: stumpS.

% Copyright 2018 Zhu Qiangjun.
% $Revision: 1.0.0.1 $  $Date: 2018/11/07 15:39:00 $


c = zeros(size(z));

ze = z(z > 0);
zh = z(z < 0); 

% z > 0, ellipse 
c(z > 0) = (1 - cos(sqrt(ze))) ./ ze;

% z == 0, parabola
c(z == 0) = 1/2;

% z < 0, hyperbola
c(z < 0) = (cosh(sqrt(-zh)) - 1) ./ (-zh);

return