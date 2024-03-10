function [coe] = State_rv_2_Orbit_Element(R, V, mu)
% 函数功能：由航天器位置速度矢量求航天器轨道六要素
% 输入：
%       R：航天器位置矢量（行向量,列向量均可，单位，km）
%       V：航天器速度矢量（行向量,列向量均可，单位km/s）。
%       muu：地球引力常量，缺省输入为398600.4415kg^3/s^2。
% 输出：
%       coe：航天器轨道六要素，具体见下（列向量）。
% ---------coe（classical orbit elements）------------- %
% a：轨道半长轴（单位，km）
% e：轨道偏心率（无量纲）
% incl：轨道倾角(单位，°)
% RAAN：升交点赤经(单位，°)
% omegap：近地点幅角(单位，°)
% TA：真近点角(单位，°)

if (2 == nargin)
	mu = 398600.4415;    %地球引力常量缺省值
end

eps = 1e-6;
r2d = 180 / pi;

R = reshape(R, 3, 1);
V = reshape(V, 3, 1);
r = norm(R);
v = norm(V);

vr = dot(R , V) / r;%径向速度
H = cross(R , V);%比角动量
h = norm(H);
incl = acos(H(3) / h);                  %轨道倾角

N = cross([0 0 1] , H);                 %真近点角
n = norm(N);

if abs(incl) <= 1e-6
    RA = 0;
elseif n ~= 0
    RA = acos(N(1) / n);
    if N(2) < 0
        RA = 2 * pi - RA;
    end
else
    RA = 0;
end

E = 1 / mu * ((v^2 - mu / r) * R - r * vr * V);%%%%%%%%%%
e = norm(E);
node_e = 1;
if abs(e) <= 1e-10
    omegap = 0;
elseif n ~= 0
    if e > eps
        omegap = real(acos(dot(N , E) / n / e));
        if E(3) < 0
            omegap = 2 * pi - omegap;
        end
    else
        omegap = 0;
    end
else
    omegap = 0;
end

if e > eps
    TA = real(acos(dot(E , R) / e / r));
    if vr < 0
        TA = 2 * pi - TA;
    end
else
    TA = real(acos(dot(N , R) / n / r));
%     if cp(3) >= 0
%         TA = real(acos(dot(N , R) / n / r));
%     else
%         TA = 2 * pi - acos(dot(N , R) / n / r);
%     end
end

a = h^2 / mu / (1 - e^2);
coe = [a ; e ; incl * r2d  ; RA * r2d ; omegap * r2d ; TA * r2d];
[r,~] = Orbit_Element_2_State_rv(coe);
if norm(r - R) > 1
    coe(6) = 360 - coe(6);
end

if e <eps && incl<=1e-6
    omegap = omegap + RA;
    RA = 0;
    TA = TA + omegap;
    omegap = 0;
    v_dir = cross([0 0 1],[1 0 0]);
    dir_proj = sign(dot(R,v_dir)/norm(v_dir)); % 如果大于，就是正的，
    TA = dir_proj* acos(dot([1,0,0],R)/norm(R));
end

end