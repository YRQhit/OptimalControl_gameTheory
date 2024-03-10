
% 初始化粒子群参数
numParticles = 50;      % 粒子数量
numVariables = 2;       % 决策变量的数量
maxIterations = 100;    % 最大迭代次数
c1 = 2;                 % 加速因子 c1
c2 = 2;                 % 加速因子 c2
w = 0.5;                % 速度权重因子 w
velocity = zeros(numParticles, numVariables);
position = rand(numParticles, numVariables);  % 初始化粒子位置

lowerBound1=-10;
lowerBound2=-10;
upperBound1=10;
upperBound2 =10;
% 定义决策变量的范围
lowerBound = [lowerBound1,lowerBound2];   % 决策变量的下界
upperBound = [upperBound1,upperBound2];   % 决策变量的上界
% 初始化个体最优解和群体最优解
pBest = position;
pBestFitness = zeros(numParticles, 1);
gBest = zeros(1, numVariables);
gBestFitness = Inf;

% 初始化图形
figure;
hold on;
title('Particle Swarm Optimization');
xlabel('Iteration');
ylabel('Fitness');

% 迭代更新粒子位置和速度
for iteration = 1:maxIterations
    % 计算适应度值并更新个体最优解和群体最优解
    for i = 1:numParticles
        fitness = MinMaxFunction(position(i, :),position(i, :));
        
        if fitness < pBestFitness(i)
            pBest(i, :) = position(i, :);
            pBestFitness(i) = fitness;
        end
        
        if fitness < gBestFitness
            gBest = position(i, :);
            gBestFitness = fitness;
        end
    end
    
    % 更新粒子速度和位置
    for i = 1:numParticles
        r1 = rand(1, numVariables);
        r2 = rand(1, numVariables);

        % 根据约束条件对速度进行限制
        velocity(i, :) = w * velocity(i, :) + c1 * r1 .* (pBest(i, :) - position(i, :)) + c2 * r2 .* (gBest - position(i, :));

        % 根据约束条件对位置进行调整
        position(i, :) = position(i, :) + velocity(i, :);

        % 处理约束条件，确保粒子位置在可行解空间内
        % 这里假设约束条件为每个决策变量的取值范围为 [lowerBound, upperBound]
        for j = 1:numVariables
            if position(i, j) < lowerBound(j)
                position(i, j) = lowerBound(j);
                velocity(i, j) = 0;  % 将速度置为0，防止越界
            elseif position(i, j) > upperBound(j)
                position(i, j) = upperBound(j);
                velocity(i, j) = 0;  % 将速度置为0，防止越界
            end
        end
    end
  % 存储迭代过程中的数据
    fitnessHistory(iteration) = gBestFitness;
    positionHistory(iteration, :, :) = position;
    
    % 绘制适应度值的变化曲线
    plot(1:iteration, fitnessHistory(1:iteration), 'b-');
    drawnow;
end

% 输出结果
disp('最优解：');
disp(gBest);
disp('最优适应度值：');
disp(gBestFitness);


% 定义MinMax问题的目标函数
function fitness = MinMaxFunction(x,u)
    % 在此定义MinMax问题的目标函数
    % 这是一个示例函数，你需要根据你的具体问题来定义目标函数
    % 这里假设是一个简单的二维函数
%     fitness = min(max(x+u));
    fitness = max(min(x+u));
end