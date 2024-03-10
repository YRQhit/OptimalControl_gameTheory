
% 初始状态向量 [x, y, z, xdot, ydot, zdot]
x0 = [1000; 200; 100; 4; 7.5; 3];
% 积分时间范围
tspan = [0 1000];
interval = 1; % 间隔为1秒
[ x_interval, y_interval, z_interval, xdot_interval, ydot_interval, zdot_interval] = solveClohessyWiltshire(x0, tspan, interval)


% 解决Clohessy-Wiltshire方程并返回结果
% 输入:
%   x0: 初始状态向量 [x, y, z, xdot, ydot, zdot]
%   tspan: 积分时间范围 [t_start, t_end]
%   interval: 目标时间间隔
% 输出:
%   x_interval, y_interval, z_interval: 每秒钟的位置变量
%   xdot_interval, ydot_interval, zdot_interval: 每秒钟的速度变量
function [ x_interval, y_interval, z_interval, xdot_interval, ydot_interval, zdot_interval] = solveClohessyWiltshire(x0, tspan, interval)
% 使用ode45求解CW方程
[t, result] = ode45(@clohessyWiltshire, tspan, x0);

% 提取位置和速度
x = result(:, 1);
y = result(:, 2);
z = result(:, 3);
xdot = result(:, 4);
ydot = result(:, 5);
zdot = result(:, 6);

% 原始结果
t_original = result(:, 1);
state_original = result(:, 1:end);

% 目标时间间隔为每秒钟

t_interval = tspan(1):interval:tspan(2);

% 使用插值方法获取每秒钟的状态变量值
state_interval = interp1(t_original, state_original, t_interval);

% 每秒钟的状态变量值
x_interval = flipud(state_interval(:, 1));
y_interval = flipud(state_interval(:, 2));
z_interval = flipud(state_interval(:, 3));
xdot_interval = flipud(state_interval(:, 4));
ydot_interval = flipud(state_interval(:, 5));
zdot_interval = flipud(state_interval(:, 6));

% 绘制位置和速度曲线
figure;
subplot(2, 1, 1);
plot(t, x, 'r', t, y, 'g', t, z, 'b');
xlabel('时间');
ylabel('位置');
legend('x', 'y', 'z');

subplot(2, 1, 2);
plot(t, xdot, 'r', t, ydot, 'g', t, zdot, 'b');
xlabel('时间');
ylabel('速度');
legend('xdot', 'ydot', 'zdot');

figure;
subplot(2, 1, 1);
plot(t_interval, x_interval, 'r', t_interval, y_interval, 'g', t_interval, z_interval, 'b');
xlabel('时间');
ylabel('位置');
legend('x', 'y', 'z');

subplot(2, 1, 2);
plot(t_interval, xdot_interval, 'r', t_interval, ydot_interval, 'g', t_interval, zdot_interval, 'b');
xlabel('时间');
ylabel('速度');
legend('xdot', 'ydot', 'zdot');
end

function dxdt = clohessyWiltshire(t, x)
% Clohessy-Wiltshire (CW) 方程
% 输入:
%   t: 时间变量（未使用，但在函数调用中需要）
%   x: 状态向量 [x, y, z, xdot, ydot, zdot]
% 输出:
%   dxdt: 状态向量的导数 [xdot, ydot, zdot, xddot, yddot, zddot]

% 定义常数

    % CW equations with external force
    mu = 3.986004418e14; % Earth's gravitational parameter (m^3/s^2)
    n = sqrt(mu / x(1)^3); % Mean motion

    A = [0      0      0      1     0     0;
         0      0      0      0     1     0;
         0      0      0      0     0     1;
         0      0      0      0     0    2*n;
         0    -n^2     0      0     0     0;
         0      0     3*n^2  -2*n   0     0];

    dxdt = A * x ;


end
