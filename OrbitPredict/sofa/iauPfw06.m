%   - - - - - - - - -
%    i a u P f w 0 6
%   - - - - - - - - -
% 
%   Precession angles, IAU 2006 (Fukushima-Williams 4-angle formulation).
% 
%   This function is part of the International Astronomical Union's
%   SOFA (Standards Of Fundamental Astronomy) software collection.
% 
%   Status:  canonical model.
% 
%   Given:
%      date1,date2     TT as a 2-part Julian Date (Note 1)
% 
%   Returned:
%      gamb            F-W angle gamma_bar (radians)
%      phib            F-W angle phi_bar (radians)
%      psib            F-W angle psi_bar (radians)
%      epsa            F-W angle epsilon_A (radians)
% 
%   Notes:
% 
%   1) The TT date date1+date2 is a Julian Date, apportioned in any
%      convenient way between the two arguments.  For example,
%      JD(TT)=2450123.7 could be expressed in any of these ways,
%      among others:
% 
%             date1          date2
% 
%          2450123.7           0.0       (JD method)
%          2451545.0       -1421.3       (J2000 method)
%          2400000.5       50123.2       (MJD method)
%          2450123.5           0.2       (date & time method)
% 
%      The JD method is the most natural and convenient to use in
%      cases where the loss of several decimal digits of resolution
%      is acceptable.  The J2000 method is best matched to the way
%      the argument is handled internally and will deliver the
%      optimum resolution.  The MJD method and the date & time methods
%      are both good compromises between resolution and convenience.
% 
%   2) Naming the following points:
% 
%            e = J2000.0 ecliptic pole,
%            p = GCRS pole,
%            E = mean ecliptic pole of date,
%      and   P = mean pole of date,
% 
%      the four Fukushima-Williams angles are as follows:
% 
%         gamb = gamma_bar = epE
%         phib = phi_bar = pE
%         psib = psi_bar = pEP
%         epsa = epsilon_A = EP
% 
%   3) The matrix representing the combined effects of frame bias and
%      precession is:
% 
%         PxB = R_1(-epsa).R_3(-psib).R_1(phib).R_3(gamb)
% 
%   4) The matrix representing the combined effects of frame bias,
%      precession and nutation is simply:
% 
%         NxPxB = R_1(-epsa-dE).R_3(-psib-dP).R_1(phib).R_3(gamb)
% 
%      where dP and dE are the nutation components with respect to the
%      ecliptic of date.
% 
%   Reference:
%      Hilton, J. et al., 2006, Celest.Mech.Dyn.Astron. 94, 351
% 
%   Called:
%      iauObl06     mean obliquity, IAU 2006
% 
%   This revision:  2009 December 17
% 
%   SOFA release 2012-03-01
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [gamb, phib, psib, epsa] = iauPfw06(date1, date2)

global const

% Interval between fundamental date J2000.0 and given date (JC).
t = ((date1 - 2451545.0) + date2) / 36525.0;

% P03 bias+precession angles
gamb = (     -0.052928     +...
       (    10.556378     +...
       (     0.4932044    +...
       (    -0.00031238   +...
       (    -0.000002788  +...
       (     0.0000000260 )...
        * t) * t) * t) * t) * t) * 4.848136811095359935899141e-6;
phib = (  84381.412819     +...
       (   -46.811016     +...
       (     0.0511268    +...
       (     0.00053289   +...
       (    -0.000000440  +...
       (    -0.0000000176 )...
        * t) * t) * t) * t) * t) * 4.848136811095359935899141e-6;
psib = (     -0.041775     +...
       (  5038.481484     +...
       (     1.5584175    +...
       (    -0.00018522   +...
       (    -0.000026452  +...
       (    -0.0000000148 )...
        * t) * t) * t) * t) * t) * 4.848136811095359935899141e-6;
epsa =  iauObl06(date1, date2);

