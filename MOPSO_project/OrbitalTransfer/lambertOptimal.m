% 最优兰伯特交会（考虑多圈及轨道方向）
% 输入：rv1 - 起始位置速度     rv2 - 交会点位置速度     T - 转移时间
% 输出：deltv1、deltv2 - 两个速度脉冲
%     x1 = [6650.39931400024;1754.89701045117;309.435691307166;-1.96930573644871;7.23789289943329;1.27623580164831];
%     x2 = [5462.22089065004;-4107.50473261990;-724.263907748599;4.62187480854522;5.96090611968822;1.05106857837123];
%     [deltv1,deltv2] = lambertOptimal(x1, x2, 2.719771697540049e+04)
function [deltv1,deltv2] = lambertOptimal(rv1, rv2, T)
global GM_Earth;
R = rv1(1:3); V = rv1(4:6);
r = norm(R);  v = norm(V);
vr = dot(R , V) / r;
H = cross(R , V);
h = norm(H);

E = 1 / GM_Earth * ((v^2 - GM_Earth / r) * R - r * vr * V);
e = norm(E);
a = h^2 / GM_Earth / (1 - e^2);
Tperiod = 2 * pi * sqrt(a^3 / GM_Earth);
Nmax = ceil(T / Tperiod); 

Dv = realmax;    
deltv1 = realmax;      deltv2 = realmax;
for i = 0:Nmax
    for k = 1:2
        if k == 2
            typle = 'long';
        else
            typle = 'short';
        end

        [V1, V2] = lamberthigh(rv1(1:3)', rv2(1:3)', T, i, GM_Earth, typle);  
        V1 = V1'; 
        V2 = V2';
        
        for j = 1:size(V1,2)
            if norm(maneuvering(R, V1 ,T) - rv2(1:3)) < 1
                dv1 = V1 - V; 
                dv2 = rv2(4:6) - V2;
                if Dv > norm(dv1) + norm(dv2)
                   deltv1 = dv1;  deltv2 = dv2;
                   Dv = norm(dv1) + norm(dv2);
                end
            end
        end
    end
end
end

function pos = maneuvering(r, v ,T)
global GM_Earth;
coe = State_rv_2_Orbit_Element(r, v, GM_Earth);
coe = twoBodyOrbit(coe, T);
b = coe(1)*sqrt(1-coe(2)^2);
if b < 6378
    pos = [0 0 0]';
else
    [pos,~] = Orbit_Element_2_State_rv(coe , GM_Earth);
end
end





