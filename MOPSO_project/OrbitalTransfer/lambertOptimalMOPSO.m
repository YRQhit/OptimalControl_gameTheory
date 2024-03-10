% 最优兰伯特交会，时间最短，
% 输入：rv1 - 起始位置速度     rv2 - 交会点位置速度     T - 转移时间
% 输出：deltv1、deltv2 - 两个速度脉冲
function [deltv1, deltv2, T] = lambertOptimalMOPSO(rv1, rv2, maxDeltv)
global GM_Earth;
R = rv1(1:3); V = rv1(4:6);
r = norm(R); v = norm(V);
vr = dot(R, V) / r;
H = cross(R, V);
h = norm(H);

E = 1 / GM_Earth * ((v^2 - GM_Earth / r) * R - r * vr * V);
e = norm(E);
a = h^2 / GM_Earth / (1 - e^2);
Tperiod = 2 * pi * sqrt(a^3 / GM_Earth);

Dv = realmax;
deltv1 = realmax;
deltv2 = realmax;

% Particle Swarm Optimization parameters
numParticles = 50;
maxIterations = 100;
w = 0.7;  % inertia weight
c1 = 1.4; % cognitive weight
c2 = 1.4; % social weight

% Initialize particle positions and velocities
positions = rand(numParticles, 1) * Tperiod;
velocities = zeros(numParticles, 1);

% Initialize global best position and value
globalBestPosition = positions(1);
globalBestValue = Dv;

% Initialize variables for plotting
fitnessHistory = zeros(maxIterations, numParticles);
globalBestHistory = zeros(maxIterations, 1);

% Perform particle swarm optimization
for iteration = 1:maxIterations
    for particle = 1:numParticles
        % Evaluate fitness of current particle
        T = positions(particle);
        
        % Check if T exceeds Tperiod
        if T > Tperiod
            T = Tperiod;
        end
        
        % Perform Lambert's problem
        [deltv1_temp, deltv2_temp] = lambertProblem(rv1, rv2, T, maxDeltv);
        
        % Calculate fitness value
        fitness = norm(deltv1_temp) + norm(deltv2_temp);
        
        % Update personal best position and value
        if fitness < Dv
            Dv = fitness;
            deltv1 = deltv1_temp;
            deltv2 = deltv2_temp;
            positions(particle) = T;
            
            if Dv < globalBestValue
                globalBestPosition = T;
                globalBestValue = Dv;
            end
        end
        
        % Store fitness values for plotting
        fitnessHistory(iteration, particle) = fitness;
    end
    
    % Store global best value for plotting
    globalBestHistory(iteration) = globalBestValue;
    
    % Update particle velocities and positions
    velocities = w * velocities + c1 * rand() * (positions - positions) + c2 * rand() * (globalBestPosition - positions);
    positions = positions + velocities;
    
    % Clamp positions within valid range
    positions(positions < 0) = 0;
    positions(positions > Tperiod) = Tperiod;
end

% Set final T to global best position
T = globalBestPosition;

% Plot fitness history
figure;
plot(1:maxIterations, globalBestHistory, 'b-', 'LineWidth', 2);
hold on;
for particle = 1:numParticles
    plot(1:maxIterations, fitnessHistory(:, particle), 'Color', [0.7 0.7 0.7]);
end
xlabel('Iteration');
ylabel('Fitness');
title('Particle Swarm Optimization');
legend('Global Best', 'Particles');
grid on;
hold off;

end


function [deltv1, deltv2] = lambertProblem(rv1, rv2, T, maxDeltv)
global GM_Earth;
R = rv1(1:3); V = rv1(4:6);
r = norm(R); v = norm(V);
vr = dot(R, V) / r;
H = cross(R, V);
h = norm(H);

E = 1 / GM_Earth * ((v^2 - GM_Earth / r) * R - r * vr * V);
e = norm(E);
a = h^2 / GM_Earth / (1 - e^2);
Tperiod = 2 * pi * sqrt(a^3 / GM_Earth);
Nmax = ceil(T / Tperiod);

Dv = realmax;
deltv1 = realmax;
deltv2 = realmax;

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

        for j = 1:size(V1, 2)
            if norm(maneuvering(R, V1, T) - rv2(1:3)) < 1 && norm(V1 - V) <= maxDeltv && norm(rv2(4:6) - V2) <= maxDeltv
                dv1 = V1 - V;
                dv2 = rv2(4:6) - V2;
                if Dv > norm(dv1) + norm(dv2)
                    deltv1 = dv1;
                    deltv2 = dv2;
                    Dv = norm(dv1) + norm(dv2);
                end
            end
        end
    end
end

end

function pos = maneuvering(r, v, T)
global GM_Earth;
coe = State_rv_2_Orbit_Element(r, v, GM_Earth);
coe = twoBodyOrbit(coe, T);
b = coe(1) * sqrt(1 - coe(2)^2);
if b < 6378
    pos = [0 0 0]';
else
    [pos, ~] = Orbit_Element_2_State_rv(coe, GM_Earth);
end
end
