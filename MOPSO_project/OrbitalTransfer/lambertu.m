function [V1, V2, Flag] = lambertu(R1, R2, t, type, rev, mu)
% lambertu  Solve Lambert's problem using universal variables.
% 
% 
% mu - gravitational parameter (km?3/s?2)
% R1, R2 - initial and final position vectors (km)
% r1, r2 - magnitudes of R1 and R2
% t - the time of flight from R1 to R2
% (a constant) (s)
% V1, V2 - initial and final velocity vectors (km/s)
% c12 - cross product of R1 into R2
% theta - angle between R1 and R2
% string - 'pro' if the orbit is prograde
% 'retro' if the orbit is retrograde
% A - a constant given by Equation 5.35
% z - alpha*x?2, where alpha is the reciprocal of the
% semimajor axis and x is the universal anomaly
% y(z) - a function of z given by Equation 5.38
% F(z,t) - a function of the variable z and constant t,
% given by Equation 5.40
% dFdz(z) - the derivative of F(z,t), given by
% Equation 5.43
% ratio - F/dFdz
% tol - tolerance on precision of convergence
% nmax - maximum number of iterations of Newton’s
% procedure
% f, g - Lagrange coefficients
% gdot - time derivative of g
% C(z), S(z) - Stumpff functions
% dum - a dummy variable
%
% User M-functions required: stumpC and stumpS
% -----------------------------------------------------------
Flag=0;
flag=0;
FLAG=0;
% Set default input values
if nargin < 6
    mu = 3.98600436e14; % Earth gravitational parameter  (m^3/s^2)
end

if nargin < 5
   rev = 0;  
end

if nargin < 4
   type = 'pro';
end

%...Magnitudes of R1 and R2:
r1 = norm(R1);
r2 = norm(R2);
c12 = cross(R1, R2);
theta = acos(dot(R1,R2)/r1/r2);

%...Determine whether the orbit is prograde or retrograde:
if ~(strcmp(type, 'pro') || strcmp(type,'retro'))
    type = 'pro';
    fprintf('\n ** Prograde trajectory assumed.\n')
end

if strcmp(type, 'pro')
    if c12(3) <= 0
        theta = 2*pi - theta;
    end
else %if strcmp(type,'retro')
    if c12(3) >= 0
        theta = 2*pi - theta;
    end
end


%...Equation 5.35:
A = sin(theta) * sqrt(r1 * r2 / (1 - cos(theta)));
if((theta-2*pi<0.000001 && theta-2*pi>-0.000001) || (theta<0.000001 && theta>-0.000001))
    V1=0;
    V2=0;
    Flag=1;
    return
end
    
if rev == 0
    % 确定隔根区间[za, zb]
    z = [38 10 1 0 -1 -10 -100 -1000 -1e4 -1e10 -1e100];
    [Fz, yz] = F(z,r1,r2,A);
    ia = find(yz>0, 1, 'last');
    ib = find(yz<0, 1, 'first'); % z>=zmin, 其中y(zmin)=0
    if ~isempty(ib)
        % 二分法求y(z) = 0
        tol = 1e-10;
        [FLAG,zmin] = bisection(@(x) y(x,r1,r2,A),z(ib), z(ia), tol );
        za = zmin;
    else
        ind = find(Fz<sqrt(mu)*t, 1, 'first');
        if isempty(ind)
            printf('\n varable z is too small.');
            za = z(end)*1e3;
        else
            za = z(ind);
        end
    end 
    zb = (2*pi)^2 - 1e-10;
    % 二分法求根
    tol = 1e-10;
   %disp(2); 
   %disp(r1); 
   %disp(r2); 
   %disp(A); 
   %disp(za); 
   %disp(zb); 
   
    [flag,z] = bisection(@(x) (F(x,r1,r2,A) - sqrt(mu) * t), za, zb, tol );   
    if (flag==1)
        V1=0;
        V2=0;
        Flag=1;
        return
    end
    [Fz,yz] = F(z,r1,r2,A); 
    % 求Lagrange系数    
    f = 1 - yz/r1;   
    g = A*sqrt(yz/mu);   
    gdot = 1 - yz/r2;
    % 求速度    
    V1 = 1/g*(R2 - f*R1);    
    V2 = 1/g*(gdot*R2 - R1);
    
    V1 = V1(:);
    V2 = V2(:); 
else
   % 多圈lambert问题 
   % 变量z在区间[(2*pi*(rev)).^2, (2*pi*(rev+1)).^2]内
   % 求出F(z)的极小值点zmin，即dFdz(z)=0的根，导函数为单调增函数，
   % 隔根区间
%    z = (2*pi*(rev+linspace(0,1,101))).^2;
%    dFz = dFdz(z,r1,r2,A);
%    ia = find(dFz<0, 1, 'last');
%    ib = find(dFz>0, 1, 'first'); 
%    za = z(ia);
%    zb = z(ib);   
   
   ia =[];
   ib =[];
   z1 = (2*pi*(rev)).^2;
   z2 = (2*pi*(rev+1)).^2;
   while isempty(ia) || isempty(ib)
       z = linspace(z1,z2,101);
       dFz = dFdz(z,r1,r2,A);
       ia = find(dFz<0, 1, 'last');
       ib = find(dFz>0, 1, 'first');
       if ~isempty(ia)
           z1 = z(ia);
       end
       if ~isempty(ib)
           z2 = z(ib);
       end
   end
   za = z(ia);
   zb = z(ib);
   
   % 二分法求dFdz(z)=0
   tol = 1e-10;
   [FLAG,zmin] = bisection(@(x) dFdz(x,r1,r2,A), za, zb, tol );
    if (FLAG==1)
        V1=0;
        V2=0;
        Flag=1;
        return
    end
   [Fzmin, yz]= F(zmin,r1,r2,A);
   if Fzmin == sqrt(mu) * t % 唯一解
       %...Equation 5.46a:
       f = 1 - yz/r1;
       %...Equation 5.46b:
       g = A*sqrt(yz/mu);
       %...Equation 5.46d:
       gdot = 1 - yz/r2;
       %...Equation 5.28:
       V1 = 1/g*(R2 - f*R1);
       %...Equation 5.29:
       V2 = 1/g*(gdot*R2 - R1);
       V1 = V1(:);
       V2 = V2(:);       
   elseif Fzmin < sqrt(mu) * t  % 有两个解  
       % left part
       zb = zmin;
       za = (2*pi*(rev)).^2 + 1e-8;
       % 二分法求根
       tol = 1e-10;
       [FLAG,z] = bisection(@(x) (F(x,r1,r2,A) - sqrt(mu) * t), za, zb, tol );
       if (FLAG==1)
            V1=0;
            V2=0;
            Flag=1;
            return
       end
       [Fz,yz] = F(z,r1,r2,A);
       %...Equation 5.46a:
       f = 1 - yz/r1;
       %...Equation 5.46b:
       g = A*sqrt(yz/mu);
       %...Equation 5.46d:
       gdot = 1 - yz/r2;
       %...Equation 5.28:
       V1L = 1/g*(R2 - f*R1);
       %...Equation 5.29:
       V2L = 1/g*(gdot*R2 - R1);
       
       % right part
       za = zmin;
       zb = (2*pi*(rev+1)).^2 - 1e-8;
       % 二分法求根
       tol = 1e-10;
       [FLAG,z] = bisection(@(x) (F(x,r1,r2,A) - sqrt(mu) * t), za, zb, tol );
       if (FLAG==1)
            V1=0;
            V2=0;
            Flag=1;
            return
       end
       [Fz,yz] = F(z,r1,r2,A);
       %...Equation 5.46a:
       f = 1 - yz/r1;
       %...Equation 5.46b:
       g = A*sqrt(yz/mu);
       %...Equation 5.46d:
       gdot = 1 - yz/r2;
       %...Equation 5.28:
       V1R = 1/g*(R2 - f*R1);
       %...Equation 5.29:
       V2R = 1/g*(gdot*R2 - R1);
       
       V1 = [V1L(:),V1R(:)];
       V2 = [V2L(:),V2R(:)];
   else % 无解
       V1=0;
       V2=0;
       Flag=1; 
       %fprintf('\n\n **No solution for n revolution, while n = %g \n\n', rev);       
   end
   
end
return

% Subfunctions used in the main body:
function [yz,cz,sz] = y(z,r1,r2,A)
cz = stumpC(z);
sz = stumpS(z);
yz = r1 + r2 + A*(z.*sz - 1) ./ sqrt(cz);
return

%...Equation 5.40:
function [Fz,yz] = F(z,r1,r2,A)
[yz,cz,sz] = y(z,r1,r2,A);
Fz = (yz./cz).^1.5 .* sz + A * sqrt(yz);
return

%...Equation 5.43:
function [dFz,yz] = dFdz(z,r1,r2,A)
[yz,cz,sz] = y(z,r1,r2,A);

dFz = (yz./cz).^1.5 .* (1/2 * (cz - 3/2* sz./cz) ./ z ...
    + 3*sz.^2/4./cz) ...
    + A/8*(3*sz./cz.*sqrt(yz) ...
    + A*sqrt(cz./yz));

yz0 = yz(z==0);
dFz(z==0) = sqrt(2)/40 * yz0.^1.5 + A/8 * (sqrt(yz0) ...
    + A * sqrt(1/2*ones(size(yz0))./yz0));
return


