function [coe] = State_rv_2_Orbit_Element(R, V, mu)
% �������ܣ��ɺ�����λ���ٶ�ʸ�������������Ҫ��
% ���룺
%       R��������λ��ʸ����������,���������ɣ���λ��km��
%       V���������ٶ�ʸ����������,���������ɣ���λkm/s����
%       muu����������������ȱʡ����Ϊ398600.4415kg^3/s^2��
% �����
%       coe�������������Ҫ�أ�������£�����������
% ---------coe��classical orbit elements��------------- %
% a������볤�ᣨ��λ��km��
% e�����ƫ���ʣ������٣�
% incl��������(��λ����)
% RAAN��������ྭ(��λ����)
% omegap�����ص����(��λ����)
% TA��������(��λ����)

if (2 == nargin)
	mu = 398600.4415;    %������������ȱʡֵ
end

eps = 1e-6;
r2d = 180 / pi;

R = reshape(R, 3, 1);
V = reshape(V, 3, 1);
r = norm(R);
v = norm(V);

vr = dot(R , V) / r;%�����ٶ�
H = cross(R , V);%�ȽǶ���
h = norm(H);
incl = acos(H(3) / h);                  %������

N = cross([0 0 1] , H);                 %������
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
    dir_proj = sign(dot(R,v_dir)/norm(v_dir)); % ������ڣ��������ģ�
    TA = dir_proj* acos(dot([1,0,0],R)/norm(R));
end

end