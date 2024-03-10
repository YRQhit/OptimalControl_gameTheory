function [deltv1, deltv2, optimalTime] = lambertOptimalPSO(rv1, rv2)
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
Nmax = ceil(Tperiod);

% 粒子群算法参数
numParticles = 50;      % 粒子数量
maxIterations = 100;    % 最大迭代次数
c1 = 2;                 % 加速因子 c1
c2 = 2;                 % 加速因子 c2
w = 0.5;                % 速度权重因子 w

% 初始化粒子位置和速度
position = rand(numParticles, 1) * Tperiod;
velocity = zeros(numParticles, 1);

% 初始化个体最优解和群体最优解
pBest = position;
pBestFitness = zeros(numParticles, 1);
gBest = 0;
gBestFitness = Inf;

% 迭代更新粒子位置和速度
for iteration = 1:maxIterations
    % 计算适应度值并更新个体最优解和群体最优解
    for i = 1:numParticles
        T = position(i);
        fitness = T;
        
        if fitness < pBestFitness(i)
            pBest(i) = T;
            pBestFitness(i) = fitness;
        end
        
        if fitness < gBestFitness
            gBest = T;
            gBestFitness = fitness;
        end
    end
    
    % 更新粒子速度和位置
    for i = 1:numParticles
        r1 = rand;
        r2 = rand;

        % 根据速度权重因子和加速因子更新速度
        velocity(i) = w * velocity(i) + c1 * r1 * (pBest(i) - position(i)) + c2 * r2 * (gBest - position(i));

        % 更新粒子位置
        position(i) = position(i) + velocity(i);

        % 处理约束条件，确保粒子位置在可行解空间内
        if position(i) < 0
            position(i) = 0;
        elseif position(i) > Tperiod
            position(i) = Tperiod;
        end
    end
end

% 获取最优解和最优转移时间
optimalTime = gBest;
% 根据最优转移时间计算最优解（速度变化）
[V1, V2] = lamberthigh(rv1(1:3)', rv2(1:3)', optimalTime, 0, GM_Earth, 'short');
deltv1 = V1' - V;
deltv2 = rv2(4:6) - V2;
end
