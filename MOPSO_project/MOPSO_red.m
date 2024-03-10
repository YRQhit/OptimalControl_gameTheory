
function delta_velocity = MOPSO_red(time)
global decide_time_red;
decide_time_red = time;
% 设置参数
numParticles = 2000; % 粒子数量
maxIterations = 100; % 最大迭代次数
w = 0.5; % 惯性权重
c1 = 2; % 自我学习因子
c2 = 2; % 社会学习因子

% 初始化粒子群
particles = initializeParticles(numParticles);

% 初始化全局最优解
globalBest = particles(1);

% 迭代优化
for iteration = 1:maxIterations
    % 更新粒子的速度和位置
    particles = updateParticles(particles, globalBest, w, c1, c2);

    % 评估粒子的适应度
    particles = evaluateParticles(particles);

    % 更新全局最优解
    globalBest = updateGlobalBest(particles, globalBest);

    % 输出当前迭代的结果
%     disp(['Iteration: ', num2str(iteration)]);
%     disp('Current global best:');
%     disp(globalBest);
end
delta_velocity = globalBest.position;

end


% 初始化粒子群
function particles = initializeParticles(numParticles)

global red_max_push;
    particles = struct('position', [], 'velocity', [], 'pBest', [], 'fitness', []);
    for i = 1:numParticles
        
        position2 = randn(1,3);
        
        if norm(position2) > red_max_push
            scaling_factor = (red_max_push / norm(position2))*rand()*red_max_push;
            position2 = position2 * scaling_factor;
        end
        particles(i).position = position2;
        particles(i).velocity = zeros(1, 3);
        particles(i).pBest = particles(i).position;
        particles(i).fitness = evaluateFitness(particles(i).position);

    end
end

% 更新粒子的速度和位置
function particles = updateParticles(particles, globalBest, w, c1, c2)

global red_max_push;
    for i = 1:numel(particles)
        % 更新速度
        particles(i).velocity = w * particles(i).velocity ...
            + c1 * rand(1, 3) .* (particles(i).pBest - particles(i).position) ...
            + c2 * rand(1, 3) .* (globalBest.position - particles(i).position);
%         particles(i).velocity = w * particles(i).velocity;
        % 更新位置
        particles(i).position = particles(i).position + particles(i).velocity;
      
        position2 = particles(i).position(1:3);
        
        if norm(position2) > red_max_push
            scaling_factor = (red_max_push / norm(position2))*rand()*red_max_push;
            position2 = position2 * scaling_factor;
        end
        particles(i).position =  position2;

    end
end


% 评估粒子的适应度
function particles = evaluateParticles(particles)
    particleFitness = evaluateFitness(particles(1).pBest);
    for i = 1:numel(particles)
        particles(i).fitness = evaluateFitness(particles(i).position);

        % 更新个体最优解
        if i > 1
            if particles(i).fitness < particleFitness
                particles(i).pBest = particles(i).position;
                particleFitness = evaluateFitness(particles(i).pBest);
            else
                particles(i).pBest = particles(i-1).pBest;
                
            end
        end
    end
end


% 更新全局最优解
function globalBest = updateGlobalBest(particles, globalBest)
    for i = 1:numel(particles)
        if particles(i).fitness <= evaluateFitness(globalBest.position)
            globalBest = particles(i);
        end
    end
end


% 适应度函数（根据具体问题进行定义）
function fitness = evaluateFitness(position)
% 这里假设适应度是一个2维向量，可以根据具体问题进行定义
fitness = red_fitness(position);

end



% 目标函数2（根据具体问题进行定义）
function value = red_fitness(position)
global decide_time_red;
global Red_rv;
global Blue_rv;
global JD_startTime;

red_vx=position(1);
red_vy=position(2);
red_vz=position(3);

blue_rv = Blue_rv;
red_rv = Red_rv;
deltv_red = [red_vx ;red_vy ;red_vz];
%使用update中没有用到的时间计算
Initialtime = JD_startTime;
red_rv(4:6)=red_rv(4:6)+deltv_red;

time = decide_time_red;

try
    blue_rv_2 = twoBodyOrbitRV(blue_rv,time);
    Red_rv_2 = twoBodyOrbitRV(red_rv,time);
    distance = norm(Red_rv_2(1:3)-blue_rv_2(1:3));
    D = distance;
    value =D;
catch
    value=+inf;
end

end

%datetime转化为行向量
function vec = datetimeToVector(dt)
    % 提取年、月、日、小时、分钟和秒的向量
    yearVec = year(dt);
    monthVec = month(dt);
    dayVec = day(dt);
    hourVec = hour(dt);
    minuteVec = minute(dt);
    secondVec = second(dt);

    % 创建行向量
    vec = [yearVec, monthVec, dayVec, hourVec, minuteVec, secondVec];
end