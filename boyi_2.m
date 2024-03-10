global JD_startTime
JD_startTime = datetime([2022,1,1,0,0,0]);
global JD_endTime
JD_endTime = datetime([2022,1,1,0,20,0]);
red = [42100 0 0.1 92 0 280];
blue = [42166 0 0.1 92 0 280];
%转化为rv
global GM_Earth
%设置最大推力
global red_max_push 
global blue_max_push 
global red_max_time
global blue_max_time
global red_min_time
global blue_min_time
global solution_array
global JD_red;
global JD_blue;
global JD_red_compare;
global JD_blue_compare;
global angle_gain;
global push_gain;
JD_red_compare = JD_startTime;
JD_blue_compare = JD_startTime;
JD_red = JD_startTime;
JD_blue = JD_startTime;
red_max_push = 0.005;
blue_max_push= 0.005;
%设置最大最小决策时间
red_max_time = 360;
blue_max_time = 360;
red_min_time = 100;
blue_min_time = 100;
angle_gain = 0.001;
push_gain = 100;
%最优解
solution_array=[];

%假设对方可以知道位置信息， 假设我方策略是对对方位置信息预估采用二体
global Blue_rv
global Red_rv
global Blue_rv_old
global Red_rv_old
[Red_r,Red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
[Blue_r,Blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
Red_rv=[Red_r;Red_v];
Blue_rv=[Blue_r;Blue_v];
Blue_rv_old = Blue_rv;
Red_rv_old = Red_rv;

global red_position;
global blue_position;
red_position=[];
blue_position=[];
global fitness;
fitness = [];
% twoBodyOrbitRV(Red_rv,1) [idealDegree,actualDegree] =
% IlluminationAngle(Red_rv,Blue_rv,JD_startTime);
%优化变量是速度大小和方向，决策时间
% red_vx,red_vy,red_vz,red_T blue_vx,blue_vy,blue_vz,blue_T fitness %决策变量
MOPSO();
function [bestSolution, bestFitness] = MOPSO()
    global solution_array
    global JD_startTime
    global JD_endTime
    global fitness
    while(datetime(JD_startTime) < datetime(JD_endTime))
        % Algorithm parameter settings
        numParticles = 50; % Number of particles
        numObjectives = 2; % Number of objective functions
        maxIterations = 10; % Number of iterations
        totalTime = 60 * 60; % Total time in seconds (one month)
        numVariables = 8;

        % Initialize the particle swarm
        particles = InitializeParticles(numParticles);

        % Initialize personal best positions and global best position
        personalBestPositions = particles;
        personalBestFitnesses = Inf(numParticles, numObjectives);
        globalBestPosition = [];
        globalBestFitness = Inf(1, numObjectives);

        % Store particle positions and fitness values at each iteration
        particlePositions = zeros(numParticles, numVariables, maxIterations);
        particleFitnesses = Inf(numParticles, numObjectives, maxIterations);

        % Iteration loop
        for iteration = 1:maxIterations % Use parfor for parallel execution
            % Update personal best positions and global best position
            for i = 1:numParticles
                fitness = EvaluateFitness(particles(i, :));

                % Update personal best position
                if dominates(fitness, personalBestFitnesses(i, :))
                    personalBestPositions(i, :) = particles(i, :);
                    personalBestFitnesses(i, :) = fitness;
                end

                % Update global best position
                if dominates(fitness, globalBestFitness)
                    globalBestPosition = particles(i, :);
                    globalBestFitness = fitness;
                end
            end

            % Store current iteration's positions and fitness values
            particlePositions(:, :, iteration) = particles;
            particleFitnesses(:, :, iteration) = personalBestFitnesses;

            % Update particle positions and velocities
            for i = 1:numParticles
                particles(i, :) = UpdateParticle(particles(i, :), personalBestPositions(i, :), globalBestPosition);
            end
        end

        bestSolution = globalBestPosition;
        bestFitness = globalBestFitness;
        bestFitness
        solution_array=[solution_array;bestSolution];
        fitness = [fitness ; bestFitness];
        updateRV(bestSolution);
        % Plot the fitness values at each iteration
%         iterationNumbers = 1:maxIterations;
%画图程序
%         hold on;
%        for obj = 1:numObjectives
%         figure;
%         hold on;
%         for i = 1:numParticles
%     %         subplot(numParticles, 1, i);
%             %画出适应度函数图像
%             OneD = reshape(particleFitnesses(i, obj, :), [], 1);
%             plot(iterationNumbers, OneD);
%             xlabel('Iteration');
%             ylabel(['Objective ', num2str(obj)]);
%             title(['Fitness values of Objective ', num2str(obj), ' for Particle ', num2str(i)]);
%         end
%         hold off;
%         end



        % Plot the particle positions at the last iteration
%         lastIterationPositions = squeeze(particlePositions(:, :, end));
%         figure;
%         hold on;
%         scatter(lastIterationPositions(:, 1), lastIterationPositions(:, 2), 'red', 'filled');
%         scatter(lastIterationPositions(:, 5), lastIterationPositions(:, 6), 'blue', 'filled');
%         xlabel('X Position');
%         ylabel('Y Position');
%         title('Particle Positions at the Last Iteration');
%         legend('Red Particles', 'Blue Particles');
%         hold off;
    end
end




function particles = InitializeParticles(numParticles)
    % Initialize particle positions and velocities Adjust the
    % initialization of x, y, z, and T based on your specific problem
    % red_vx, red_vy, red_vz, red_T blue_vx, blue_vy, blue_vz, blue_T
    global red_max_push 
    global blue_max_push 
    global red_max_time
    global blue_max_time
    global red_min_time
    global blue_min_time
    numVariables = 8;
    particles = zeros(numParticles, numVariables);
    for i = 1:numParticles
        % Randomly distribute the total time among the tasks
%         totalTaskTime = totalTime; taskTimes = zeros(1, 2);
%         
%         for j = 1:2
%             if totalTaskTime <= 0
%                 break;
%             end
%             
%             taskTime = rand() * totalTaskTime; taskTimes(j) = taskTime;
%             totalTaskTime = totalTaskTime - taskTime;
%         end
        
        particles(i, 4) = red_min_time + (red_max_time-red_min_time) * rand;  
        particles(i, 8) = blue_min_time + (blue_max_time-blue_min_time) * rand;   
        % Randomly initialize velocities

        particles(i, 1:3) = randn(1, 3);
        if(norm(particles(i, 1:3))>red_max_push)
            scaling_factor = (red_max_push / norm(particles(i, 1:3)));
            particles(i, 1:3) = particles(i, 1:3)*scaling_factor;
        end
        particles(i, 5:7) = randn(1, 3);
        if(norm(particles(i, 5:7))>blue_max_push)
            scaling_factor = (blue_max_push / norm(particles(i, 5:7)));
            particles(i, 5:7) = particles(i, 5:7)*scaling_factor;
        end 
%         particles
    end
end



%EvaluateFitness求的是最小值
function fitness = EvaluateFitness(position)
    % 计算目标函数值 根据具体问题进行适当的定义
    fitness = [red_fitness(position), blue_fitness(position)];
end
function newPosition = UpdateParticle(position, personalBestPosition, globalBestPosition)
    % 更新粒子位置和速度 根据具体问题进行适当的更新操作
    global red_max_push 
    global blue_max_push 
    global red_max_time
    global blue_max_time
    global red_min_time
    global blue_min_time
    red_vx_index = 1;
    red_vy_index = 2;
    red_vz_index = 3;
    red_T_index = 4;
    blue_vx_index = 5;
    blue_vy_index = 6;
    blue_vz_index = 7;
    blue_T_index = 8;
    
    newPosition = position; % 需要根据实际问题进行修改

    % 使用个体最佳位置和全局最佳位置进行位置更新
    personalBestFactor = 0.5; % 个体最佳位置影响因子
    globalBestFactor = 0.3; % 全局最佳位置影响因子
    randomFactor = 0.2; % 随机因子

    newPosition = newPosition + personalBestFactor * rand() * (personalBestPosition - position);
    newPosition = newPosition + globalBestFactor * rand() * (globalBestPosition - position);
    newPosition = newPosition + randomFactor * rand(size(newPosition)); % 添加随机扰动
    
    % 对时间大小限制
    newPosition(red_T_index) = min(newPosition(red_T_index), red_max_time);
    newPosition(red_T_index) = max(newPosition(red_T_index), red_min_time);
    newPosition(blue_T_index) = min(newPosition(blue_T_index), blue_max_time);
    newPosition(blue_T_index) = max(newPosition(blue_T_index), blue_min_time);
    % 对速度大小限制
    red_v_squared = newPosition(red_vx_index)^2 + newPosition(red_vy_index)^2 + newPosition(red_vz_index)^2;
    blue_v_squared = newPosition(blue_vx_index)^2 + newPosition(blue_vy_index)^2 + newPosition(blue_vz_index)^2;

    % 判断并限制红方速度大小
    if red_v_squared > red_max_push^2
        scaling_factor = sqrt(red_max_push^2 / red_v_squared);
        newPosition(red_vx_index) = newPosition(red_vx_index) * scaling_factor;
        newPosition(red_vy_index) = newPosition(red_vy_index) * scaling_factor;
        newPosition(red_vz_index) = newPosition(red_vz_index) * scaling_factor;
    end
    
    % 判断并限制蓝方速度大小
    if blue_v_squared > blue_max_push^2
        scaling_factor = sqrt(blue_max_push^2 / blue_v_squared);
        newPosition(blue_vx_index) = newPosition(blue_vx_index) * scaling_factor;
        newPosition(blue_vy_index) = newPosition(blue_vy_index) * scaling_factor;
        newPosition(blue_vz_index) = newPosition(blue_vz_index) * scaling_factor;
    end
%     newPosition
end

function result = dominates(fitness1, fitness2)
    % 判断解 fitness1 是否优于解 fitness2 根据多目标优化的定义进行判断
    if size(fitness1) ~= size(fitness2)
        error('The dimensions of fitness1 and fitness2 do not match.');
    end

    result = all(fitness1 <= fitness2) && any(fitness1 < fitness2);
end



% 定义目标函数
function objectiveValue = red_fitness(position)
    % 根据具体问题定义目标函数1
    global Red_rv;
    global Blue_rv;
    global Red_rv_old;
    global Blue_rv_old;
    global JD_startTime;
    global JD_red;
    global JD_red_compare;
    global JD_blue_compare;
    global angle_gain;
    global push_gain;
     red_vx=position(1);
     red_vy=position(2);
     red_vz=position(3);
     red_T=position(4);
     blue_vx=position(5);
     blue_vy=position(6);
     blue_vz=position(7);
     blue_T=position(8);
     deltv_red = [red_vx ;red_vy ;red_vz];
     deltv_blue = [blue_vx ;blue_vy ;blue_vz];
     %获得的开始的blue位置速度
     if(JD_startTime<JD_red_compare)
         red_rv = Red_rv_old;
         blue_rv = Blue_rv;
     end
     if(JD_startTime>=JD_red_compare)
         red_rv = Red_rv;
         blue_rv = Blue_rv_old;
     end
     blue_rv = Blue_rv_old;
     red_rv = Red_rv;
     %使用update中没有用的时间计算
     Initialtime = JD_startTime;
     red_rv(4:6)=Red_rv(4:6)+deltv_red;
     J=0;
     try
         for i = 0:red_T
            %Blue飘飞计算
            blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
            Red_rv_2 = twoBodyOrbitRV(red_rv,1);
            %角度计算

            %xc追踪和xt任务星rv
            Initialtime = Initialtime + seconds (1);
            time = datetimeToVector(Initialtime);
            [~,actualDegree] = IlluminationAngle(Red_rv_2,blue_rv_2,time);
            distance = norm(Red_rv_2(1:3)-blue_rv_2(1:3));
            blue_rv = blue_rv_2;
            red_rv = Red_rv_2;
            %计算收益
            J = J + actualDegree*angle_gain+distance;
         end
     catch
        deltv_red
     
        objectiveValue = inf;
     end
    %最后增益
    objectiveValue=J +norm(deltv_red)*push_gain;
end

function objectiveValue = blue_fitness(position)
        % 根据具体问题定义目标函数2
    global Red_rv;
    global Blue_rv;
    global Red_rv_old;
    global Blue_rv_old;
    global JD_startTime;
    global JD_blue;
    global JD_red_compare;
    global JD_blue_compare;
    global angle_gain;
    global push_gain;
     red_vx=position(1);
     red_vy=position(2);
     red_vz=position(3);
     red_T=position(4);
     blue_vx=position(5);
     blue_vy=position(6);
     blue_vz=position(7);
     blue_T=position(8);
     deltv_red = [red_vx ;red_vy ;red_vz];
     deltv_blue = [blue_vx ;blue_vy ;blue_vz];
     %获得的开始的blue位置速度
     if(JD_startTime<JD_blue_compare)
         red_rv = Red_rv;
         blue_rv = Blue_rv_old;
     end
     if(JD_startTime>=JD_blue_compare)
         red_rv = Red_rv_old;
         blue_rv = Blue_rv;
     end
     
     %使用update中没有用到的时间计算   
     Initialtime = JD_startTime;
     blue_rv(4:6)=Blue_rv(4:6)+deltv_blue;
     J=0;
    try
       for i = 0:blue_T
            %Blue飘飞计算
            red_rv_2 = twoBodyOrbitRV(red_rv,1);
            blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
            %角度计算 xc追踪和xt任务星rv
            Initialtime = Initialtime + seconds (1);
            time = datetimeToVector(Initialtime);
            [idealDegree,actualDegree] = IlluminationAngle(blue_rv_2,red_rv_2,time);
            distance = norm(red_rv_2(1:3)-blue_rv_2(1:3));
            blue_rv = blue_rv_2;
            red_rv = red_rv_2;
            %计算收益
            J = J + actualDegree*angle_gain+distance ;
       end
    catch
%         blue_rv
%         red_rv
 
    deltv_blue
        objectiveValue = inf;
    end
    %最后增益
    objectiveValue=J + norm(deltv_blue)*push_gain;
    
end


function  [] = updateRV(position)
%更新速度和时间信息
global red_position;
global blue_position;
    global Red_rv;
    global Blue_rv;
    global JD_startTime;
    global Blue_rv_old;
    global Red_rv_old;
    %设置这个时间用于计算长一点的时间的转移时间
    global JD_red;
    global JD_blue;

    global JD_red_compare;
    global JD_blue_compare;
     red_T=position(4);
     blue_T=position(8);
     red_v = position(1:3);
     blue_v = position(5:7);
%      Red_rv(4:6)=Red_rv(4:6)+red_v';
%      Blue_rv(4:6)=Blue_rv(4:6)+blue_v';
%      if(blue_T>red_T)
%          Red_rv(4:6)=Red_rv(4:6)+red_v';
%          JD_startTime = JD_startTime + seconds (red_T);
%          Red_rv = twoBodyOrbitRV(Red_rv,red_T);
%          Blue_rv_old = twoBodyOrbitRV(Blue_rv,red_T);
%          Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
%          JD_blue =JD_startTime + seconds (blue_T);
%          
%      end
%      
%      if(blue_T<red_T)
%          JD_startTime = JD_startTime + seconds (blue_T);
%          Red_rv_old = twoBodyOrbitRV(Red_rv,blue_T);
%          Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
%          Red_rv = twoBodyOrbitRV(Red_rv,red_T);
%          JD_red = JD_startTime + seconds (red_T);
%      end
%开始时间第一次
try
     if(JD_blue_compare == JD_red_compare)
         Red_rv(4:6)=Red_rv(4:6)+red_v';
         Blue_rv(4:6)=Blue_rv(4:6)+blue_v';       
         if(blue_T<red_T)
             JD_startTime = JD_startTime + seconds (blue_T);
             Red_rv_old = twoBodyOrbitRV(Red_rv,blue_T);
             Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
             Red_rv = twoBodyOrbitRV(Red_rv,red_T);
             JD_red = JD_startTime + seconds (red_T);
             JD_blue_compare = JD_blue_compare+ seconds (blue_T);
             JD_red_compare = JD_red_compare+ seconds (red_T);
         end
         if(blue_T>red_T)
             JD_startTime = JD_startTime + seconds (red_T);
             Red_rv = twoBodyOrbitRV(Red_rv,red_T);
             Blue_rv_old = twoBodyOrbitRV(Blue_rv,red_T);
             Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
             JD_blue =JD_startTime + seconds (blue_T);
             JD_red_compare = JD_red_compare+ seconds (red_T);
             JD_blue_compare = JD_blue_compare+ seconds (blue_T);
         end
     
%      两种情况进行分别更新
     elseif(JD_blue_compare>JD_red_compare)
         
         JD_red_compare = JD_red_compare+seconds (red_T);
         if(JD_blue_compare>JD_red_compare)
            JD_startTime=JD_red_compare;
             Red_rv(4:6)=Red_rv(4:6)+red_v';
             Red_rv = twoBodyOrbitRV(Red_rv,red_T);
             Blue_rv_old = twoBodyOrbitRV(Blue_rv,red_T);
             JD_blue =JD_blue_compare;
         end
         if(JD_blue_compare<=JD_red_compare)
            
             JD_startTime=JD_blue_compare;
             trans_T = seconds(JD_blue_compare-(JD_red_compare-seconds (red_T)));
             Red_rv_old = twoBodyOrbitRV(Red_rv,trans_T);
             Red_rv(4:6)=Red_rv(4:6)+red_v';
             Red_rv = twoBodyOrbitRV(Red_rv,red_T);
       
         end
     
     
     elseif(JD_blue_compare<JD_red_compare)
         JD_blue_compare = JD_blue_compare+seconds (blue_T);
         if(JD_blue_compare>JD_red_compare)
            JD_startTime=JD_red_compare;
            trans_T = seconds(JD_red_compare-(JD_blue_compare-seconds (blue_T)));
            Blue_rv_old = twoBodyOrbitRV(Blue_rv,trans_T);
            Blue_rv(4:6)=Blue_rv(4:6)+blue_v';
            Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
         end
         if(JD_blue_compare<=JD_red_compare)
             JD_startTime=JD_blue_compare;
             Blue_rv(4:6)=Blue_rv(4:6)+blue_v';
             Blue_rv = twoBodyOrbitRV(Blue_rv,blue_T);
             Red_rv_old = twoBodyOrbitRV(Red_rv,blue_T);
             JD_blue =JD_blue_compare;
         end
     end
     red_position=[red_position,Red_rv_old];
     blue_position=[blue_position,Blue_rv_old];
catch
%     Blue_rv
%     blue_T
%     Red_rv
%     red_T
    blue_v
    red_v
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
