function [V1, V2] = lamberthigh(r1vec, r2vec, tf, varargin) 
%LAMBERTHIGH                    High-Thrust Lambert-targeter 
% 
%   Usage: 
%   [V1, V2] = LAMBERTHIGH(r1, r2, tf) 
%   [V1, V2] = LAMBERTHIGH(r1, r2, tf, m) 
%   [V1, V2] = LAMBERTHIGH(r1, r2, tf, m, mu) 
%   [V1, V2] = LAMBERTHIGH(r1, r2, tf, m, mu, way) 
% 
%   [V1, V2] = LAMBERTHIGH(r1, r2, tf) determines the transfer trajectories 
%   between [r1] = [x1 y1 z1] and [r2] = [x2 y2 z2], that take a transfer  
%   time [tf] in seconds.  LAMBERTHIGH returns [V1, V2], where [V1] and  
%   [V2] are the velocity vectors at [r1] and [r2], respectively. By 
%   default, all trajectories are assumed to be Heliocentric. 
% 
%   LAMBERTHIGH(r1, r2, tf, m) does the same, but then with [m] complete  
%   orbits in between. In this case, it may be that the problem has no 
%   solution for the given parameters. If that is true, the results will  
%   be NaN. [m] may be empty, e.g., [m] = [], in which case it will default 
%   to a value of zero. 
% 
%   LAMBERTHIGH(r1, r2, tf, m, mu) takes the parameter [mu] as the standard 
%   gravitational parameter of the central body. This will make the Lambert 
%   targeter suited for use around other bodies than the Sun. [mu] may be  
%   empty, e.g., [mu] = [], in which case the Sun will be used as the 
%   central body. 
% 
%   LAMBERTHIGH(r1, r2, tf, m, mu, way) allows selection of the 'long' or  
%   'short' way solution. By default, BOTH solutions are returned, which is 
%   equivalent to [way] = 'both' (see next paragraph). The argument [way]  
%   may also take values equal to 'short' or 'long', in which case only the  
%   'short' or 'long' way solutions will be returned (small- and large  
%   values of the turn angle, respectively). 
% 
%   Inputs [r1] and [r2] may be of size (n x 3), as long as the inputs [tf] 
%   and [m] are of corresponding size (n x 1). Normally, there are 2 
%   solutions to the Lambert problem (for the given turn angle, and 2pi 
%   minus that turn angle). The default outputs [V1] and [V2] are thus of  
%   size [n x 3 x 2]--the second solution is appended along the third 
%   array dimension. If only one solution is desired ('short' or 'long'),  
%   the size of the outputs [V1] and [V2] will be [n x 3].  
% 
%   LAMBERTHIGH uses the method developed by Lancaster & Blancard, as  
%   described in their 1969 paper. Initial values, and several details of  
%   the procedure, are provided by R.H. Gooding, as described in his 1990  
%   paper. Note that the derivatives near x = 0 (circular) and x = 1  
%   (parabolic) may be inaccurate, as indicated by Gooding. This issue has  
%   not yet been addressed in LAMBERTHIGH, so convergence might be an issue 
%   in those cases. 
% 
%   See also lambertlow, halley. 
 
%   Author: Rody P.S. Oldenhuis 
%   Delft University of Technology 
%   E-mail: oldenhuis@dds.nl 
%   Last edited 26/Feb/2009. 
 
    % basic errortrap 
    error(nargchk(3, 6, nargin));  % 检测输入参数的个数
     
    % constants and initial values 
    tol = 1e-6;            % require accurate results % changed by PH from 1e-7 to 1e-10
    muS = 132712439940;    %  gravitational parameter Sun (from NASA/JPL Horizons) 
     
    % extract parameters 
    zs = zeros(size(tf));   % by default, all [m] = 0 
    ms = zs; 
    whichsol = 'both';      % by default, compute both solutions 
    if (nargin >= 4) 
        ms = varargin{1};         
        if isempty(ms), ms = zs; end         
    end 
    if (nargin >= 5) % changed by PH from == to >=
        mu = varargin{2}; 
        if isempty(mu), mu = muS; end 
        muS = mu; 
    end	 
    if (nargin >= 6) 
        whichsol = varargin{3}; 
        if ~ischar(whichsol) 
            % errortrap 
            warning('lamberthigh:incorrect_sols',... 
                    ['Solutions should be selected by providing strings "both",',... 
                     ' "short" or "long". Defaulting to "both"...']); 
            whichsol = 'both'; 
        end 
        if isempty(whichsol) 
            whichsol = 'both'; 
        end 
    end 
    if strcmpi(whichsol, 'both') 
        whichsol = 0; 
        numsols  = 2; 
    elseif strcmpi(whichsol, 'short') 
        whichsol = 1; 
        numsols  = 1; 
    elseif strcmpi(whichsol, 'long') 
        whichsol = 2; 
        numsols  = 1; 
    end 
     
    % less basic errortraps 
    if (size(r1vec, 2) ~=3 || size(r2vec, 2) ~= 3) 
        error('lamberthigh:R1R2_not_3_cols', ... 
              'Radius vectors should have 3 columns.'); 
    end    
    if (size(r1vec) ~= size(r2vec)) 
        error('lamberthigh:incorrect_size_R1R2', ... 
              'Radius vectors should be the same size.'); 
    end     
    if (size(r1vec, 1) ~= size(tf, 1)) 
        error('lamberthigh:incorrect_size_tf', ... 
              'Time-of-flight vector should have as many rows as [r1] and [r2].'); 
    end     
    if (size(ms) ~= size(tf)) 
        error('lamberthigh:incorrect_size_m', ... 
              '[m]-vector vector should have as many rows as [r1] and [r2].'); 
    end 
         
    % manipulate input 
    nvecs   = size(r1vec, 1);             % 位置向量行数
    r1      = sqrt(sum(r1vec.^2, 2));     % 起点到引力中心距离
    r2      = sqrt(sum(r2vec.^2, 2));     % 终点到引力中心距离
    r1unit  = r1vec ./ [r1, r1, r1];      % 起点的单位矢量
    r2unit  = r2vec ./ [r2, r2, r2];      % 终点的单位矢量
    dotprod = sum(r1unit.*r2unit, 2);     % 向量点乘
    crsprod = cross(r1vec, r2vec, 2);     % 向量叉乘
    mcrsprd = sqrt(sum(crsprod.^2, 2)); 
    th1unit = cross(crsprod./[mcrsprd, mcrsprd, mcrsprd], r1unit, 2);    
    th2unit = cross(crsprod./[mcrsprd, mcrsprd, mcrsprd], r2unit, 2); 
         
    % heliocentric angle(s) between target and departure 轨道夹角   
    dth = real(acos(dotprod));            % small turn angle  % changed by PH by adding real()
    if (whichsol == 2)               
        dth = dth - 2*pi;           % but use only large turn angle  优弧转移 
    elseif  (whichsol == 0) 
        dth = [dth, dth - 2*pi];    % use both large and small turn angles 
    end 
     
    % initial output is pessimistic 
    V1 = NaN(size(r1vec, 1), 3, numsols); 
    V2 = V1; 
     
    % for every pair of vectors in the input 
    for pair = 1:nvecs         
 
        % shorter notation 
        r1p  = r1(pair); 
        r2p  = r2(pair);   
        m    = ms(pair); 
         
        % don't skip the solutions yet 
        skip = false; 
             
        % number of solutions 
        for sol = 1:numsols 
                 
            % define constants for this pair/turn angle 
            c  = sqrt(r1p^2 + r2p^2 - 2*r1p*r2p*cos(dth(pair, sol))); 
            s  = (r1p + r2p + c) / 2; 
            T  = sqrt(8.*muS./s.^3) * tf(pair); 
            q  = sqrt(r1p.*r2p)./s * cos(dth(pair, sol)/2); 
 
            % general formulae for the initial values (Gooding) 
            if ~skip 
                T0  = Tx(0); 
                Td  = T0 - T;  
                x01 = T0*Td/4/T; 
                if (Td > 0) && (m == 0) 
                    x0t = x01; 
                else 
                    phr = mod(2*atan2(2*q, 1 - q^2), 2*pi); 
                    x01 = Td/(4 - Td); 
                    x02 = -sqrt( -Td/(T+T0/2) ); 
                    W   = x01 + 1.7*sqrt(2 - phr/pi); 
                    if (W >= 0) 
                        x03 = x01; 
                    else                           
                        x03 = x01 + (-W).^(1/16).*(x02 - x01); 
                    end 
                    lambda = 1 + x03*(1 + x01)/2 - 0.03*x03^2*sqrt(1 + x01); 
                    x02    = lambda*x03; 
                    x0t    = x02; 
                end 
 
                % single-revolution case 
                if (m == 0) 
                    x0 = [x0t, x0t]; 
                 
                % multi-revolution case     
                else 
 
                    % determine minimum Tp(x) 
                    xMpi = 4/(3*pi*(2*m + 1)); 
                    if (phr < pi) 
                        xM0 = xMpi.*(phr/pi)^(1/8); 
                    elseif (phr > pi) 
                        xM0 = xMpi.*(2 - (2 - phr/pi)^(1/8)); 
                    end 
                    xM = halley(@Tp, @Tpp, @Tppp, xM0, tol); 
                    TM = Tx(xM); 
 
                    % if there is no solution, continue with next vector pair 
                    if (TM > T) 
                        break 
                    end 
 
                    % set different end-conditions 
                    skip = true; 
                    x0   = [x01, x02]; 
 
                end 
            end 
 
            % find root of Lancaster & Blancard's function   
            x = halley(@(y) Tx(y) - T, @Tp, @Tpp, x0(sol), tol, -1, inf); 
 
            % calculate terminal velocities 
             
            % constants required for this calculation 
            gamma = sqrt(muS*s/2); 
            rho   = (r1p - r2p)/c; 
            sigma = 2*sqrt(r1p*r2p/(c^2)) * sin(dth(pair, sol)/2);     
            z     = sqrt(1 - q^2 + q^2*x^2); 
 
            % radial component 
            Vr1    = +gamma*((q*z - x) - rho*(q*z + x)) / r1p;         
            Vr1vec = Vr1(:, [1, 1, 1]) .* r1unit(pair, :); 
            Vr2    = -gamma*((q*z - x) + rho*(q*z + x)) / r2p; 
            Vr2vec = Vr2(:, [1, 1, 1]) .* r2unit(pair, :); 
 
            % tangential component 
            Vtan1      = sigma * gamma * (z + q*x) / r1p;             
            Vtan1vec   = Vtan1(:, [1, 1, 1]) .* th1unit(pair, :); 
            Vtan2      = sigma * gamma * (z + q*x) / r2p; 
            Vtan2vec   = Vtan2(:, [1, 1, 1]) .* th2unit(pair, :); 
 
            % Cartesian velocity 
            V1(pair, :, sol) = Vtan1vec + Vr1vec; 
            V2(pair, :, sol) = Vtan2vec + Vr2vec;             
             
        end         
    end 
     
    % Lancaster & Blanchard's function     
    function Tx = Tx(x)  
        E  = x^2 - 1;         
        y  = sqrt(abs(E));         
        z  = sqrt(1 - q^2 + q^2*x^2 );         
        f  = y*(z - q*x); 
        g  = x*z - q*E;         
        if (E < 0);                
            d = atan2(f, g) + 2*pi*m; 
        else 
            d = log(f + g); 
        end 
        Tx = 2*(x - q*z - d/y)/E;         
    end 
 
    % first derivative (from Lancaster & Blanchard, 1969) 
    function Tpx = Tp(x) 
        z = sqrt(1 - q^2 + q^2*x^2 ); 
        E = x^2 - 1; 
        Tpx = (4 - 4*q^3*x/z - 3*x*Tx(x))/E; 
    end 
     
    % second derivative (derivative from first derivative) 
    function Tppx = Tpp(x) 
        z = sqrt(1 - q^2 + q^2*x^2 ); 
        E = x^2 - 1; 
        Tppx = (-4*q^3/z * (1 - q^2*x^2/z^2) - 3*Tx(x) - 3*x*Tp(x))/E;         
    end 
     
    % third derivative (derivative from second derivative) 
    function Tpppx = Tppp(x) 
        z = sqrt(1 - q^2 + q^2*x^2 ); 
        E = x^2 - 1; 
        Tpppx = (4*q^3/z^2*((1 - q^2*x^2/z^2) + 2*q^2*x/z^2*(z - x)) - ... 
                8*Tp(x) - 7*x*Tpp(x))/E;            
    end 
     
end