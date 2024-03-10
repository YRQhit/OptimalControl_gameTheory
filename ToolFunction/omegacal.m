function [omega] = omegacal(r,mu)
% OMEGACAL 用于计算圆轨道的角速度可以一次计算多个，以列向量的形式
% r为圆轨道的半径,标量，输入一列，意味着计算多个轨道半径的角速度，单位是km
% mu 为引力常量，缺省输入为398600.4415

% 2021 0426

if nargin == 1
    mu = 398600.4415;
end

omega = sqrt(mu./(r.^3));

end