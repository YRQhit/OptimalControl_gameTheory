% 多目标粒子群优化算法框架
global red_max_push;
global blue_max_push;
global Blue_rv
global Red_rv
global red_position;
global blue_position;
global decide_time;
decide_time = 30;
global predict_time;
predict_time = 30;
red_max_push = 0.05;
blue_max_push  = 0.05;
global JD_startTime
global JD_endTime
JD_startTime = datetime([2022,1,1,0,0,0]);
JD_endTime = datetime([2022,1,1,0,30,0]);

red = [42100 0 0.1 92 0 280];
blue = [42166 0 0.1 92 0 280];

[Red_r,Red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
[Blue_r,Blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
Red_rv=[Red_r;Red_v];
Blue_rv=[Blue_r;Blue_v];
red_position = [Red_rv];
blue_position = [Blue_rv];

MOPSO();
function [] = MOPSO()
global JD_startTime
global JD_endTime
% 设置参数
numParticles = 10; % 粒子数量
maxIterations = 10; % 最大迭代次数
w = 0.5; % 惯性权重
c1 = 2; % 自我学习因子
c2 = 2; % 社会学习因子
while(JD_startTime<JD_endTime)
    globalBest = [];
    % 迭代优化
    for iteration = 1:maxIterations
        % 初始化粒子群
        particles = initializeParticles(numParticles);

        % 初始化全局最优解
        globalBest = particles(1);
    
        % 更新粒子的速度和位置
        particles = updateParticles(particles, globalBest, w, c1, c2);
        
        % 评估粒子的适应度
        particles = evaluateParticles(particles);
        
        % 更新全局最优解
        globalBest = updateGlobalBest(particles, globalBest);
%         globalBest.position
        % 输出当前迭代的结果
        %     disp(['Iteration: ', num2str(iteration)]);
        %     disp('Current global best:');
        %     disp(globalBest);
        
    end
    updateRV(globalBest.position);
end
end
% 初始化粒子群
function particles = initializeParticles(numParticles)

% 在适当的范围内随机初始化粒子的位置和速度 这里假设每个粒子的位置是一个6维向量，速度也是一个6维向量

particles = struct('position', [], 'velocity', [], 'pBest', [], 'fitness', []);
%为了使用并行计算使用真实值
for i = 1:numParticles
    position1 = randn(1,3);
    position2 = randn(1,3);
    if norm(position1)>0.05
        scaling_factor = (0.05 / norm(position1));
        position1 = position1*scaling_factor;
    end
    if norm(position2)>0.05
        scaling_factor = (0.05 / norm(position2));
        position2 = position2*scaling_factor;
    end
    particles(i).position = [position1 , position2];
    particles(i).velocity = zeros(1, 6);
    particles(i).pBest = particles(i).position;
    particles(i).fitness = evaluateFitness(particles(i).position);
end
end


% 更新粒子的速度和位置
function particles = updateParticles(particles, globalBest, w, c1, c2)

for i = 1:numel(particles)
    % 更新速度
    particles(i).velocity = w * particles(i).velocity ...
        + c1 * rand(1, 6) .* (particles(i).pBest - particles(i).position) ...
        + c2 * rand(1, 6) .* (globalBest.position - particles(i).position);
    
    % 更新位置
    particles(i).position = particles(i).position + particles(i).velocity;
    position1 = particles(i).position(1:3);
    position2 = particles(i).position(4:6);
    %为了可以并行计算，使用真实数据
    if norm(position1)>0.05
        scaling_factor = (0.05 / norm(position1));
        position1 = position1*scaling_factor;
    end
    if norm(position2)>0.05
        scaling_factor = (0.05 / norm(position2));
        position2 = position2*scaling_factor;
    end
    particles(i).position = [position1,position2];
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
fitness = [red_fitness(position), blue_fitness(position)];
end

% 目标函数1（根据具体问题进行定义）
function value = red_fitness(position)
global decide_time;
global Red_rv;
global Blue_rv;

global JD_startTime;
red_vx=position(1);
red_vy=position(2);
red_vz=position(3);

blue_vx=position(4);
blue_vy=position(5);
blue_vz=position(6);
blue_rv = Blue_rv;
red_rv = Red_rv;
deltv_red = [red_vx ;red_vy ;red_vz];
deltv_blue = [blue_vx ;blue_vy ;blue_vz];
%使用update中没有用的时间计算
Initialtime = JD_startTime;
red_rv(4:6)=red_rv(4:6)+deltv_red;

J=0;

for i = 0:decide_time-1
    %Blue飘飞计算
    blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
    Red_rv_2 = twoBodyOrbitRV(red_rv,1);
    %角度计算
    
    %xc追踪和xt任务星rv
    Initialtime = Initialtime + seconds (1);
    time = datetimeToVector(Initialtime);
%     [~,actualDegree] = IlluminationAngle(Red_rv_2,blue_rv_2,time);
    distance = norm(Red_rv_2(1:3)-blue_rv_2(1:3));
    blue_rv = blue_rv_2;
    red_rv = Red_rv_2;
    
    %计算收益
    J = J +distance;
    
%       J = J + actualDegree+abs(distance);
end
value =J;
end

% 目标函数2（根据具体问题进行定义）
function value = blue_fitness(position)
global decide_time;
global Red_rv;
global Blue_rv;
global JD_startTime;
red_vx=position(1);
red_vy=position(2);
red_vz=position(3);

blue_vx=position(4);
blue_vy=position(5);
blue_vz=position(6);

blue_rv = Blue_rv;
red_rv = Red_rv;
deltv_red = [red_vx ;red_vy ;red_vz];
deltv_blue = [blue_vx ;blue_vy ;blue_vz];
%使用update中没有用到的时间计算
Initialtime = JD_startTime;
blue_rv(4:6)=blue_rv(4:6)+deltv_blue;
J=0;
for i = 0:decide_time-1
    %Blue飘飞计算
    red_rv_2 = twoBodyOrbitRV(red_rv,1);
    blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
    %角度计算 xc追踪和xt任务星rv
    Initialtime = Initialtime + seconds (1);
    time = datetimeToVector(Initialtime);
%     [idealDegree,actualDegree] = IlluminationAngle(blue_rv_2,red_rv_2,time);
    distance = norm(red_rv_2(1:3)-blue_rv_2(1:3));
    blue_rv = blue_rv_2;
    red_rv = red_rv_2;
    %计算收益
    J = J +distance;
%     J = J + actualDegree+abs(distance) ;
end
value =J;
end

function  [] = updateRV(position)
%更新速度和时间信息
global Red_rv;
global Blue_rv;
global red_position;
global blue_position;
global JD_startTime;
global predict_time;
global decide_time;
red_v = position(1:3);
blue_v = position(4:6);
Red_rv(4:6)=Red_rv(4:6)+red_v';
Blue_rv(4:6)=Blue_rv(4:6)+blue_v';

Red_rv = twoBodyOrbitRV(Red_rv,predict_time);
Blue_rv = twoBodyOrbitRV(Blue_rv,predict_time);
distance = norm(Red_rv(1:3)-Blue_rv(1:3))
% if  distance<20
%     predict_time = predict_time/2;
%     decide_time = decide_time/2;
% end
% if  distance<10
%     predict_time = predict_time/4;
%     decide_time = decide_time/4;
% end
% if  distance<1
%     predict_time = 1;
%     decide_time = 1;
% end
red_position=[red_position,Red_rv];
blue_position=[blue_position,Blue_rv];
JD_startTime  = JD_startTime+seconds (predict_time);

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
