function dv_all = fuelmass2dv(fuel,md,I,g,eta)
% ���dv_all ��λm/s

if nargin == 1 
    md = 1450;
    I = 285;
    g = 9.8;
    eta = 0.999;
end

dv_all = -log(1-eta*fuel/md)*I*g;

end