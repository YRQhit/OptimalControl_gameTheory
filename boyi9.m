% 设置初始状态向量
x0 = [7000; 600; 7000; 90; 80; 70];

% 设置时间范围和时间步长
tspan = [0 360];
dt = 1;

% 设置外部力
F_ext = [0; 0; 0];

% 使用 ode45 函数求解 CW 方程（包括外部力）
[t, result] = ode45(@(t,x) clohessy_wiltshire_external(t,r, x, F_ext), tspan, x0);
% [t, result] = ode45(@clohessy_wiltshire, tspan, x0);
% x = result(:, 1);
% y = result(:, 2);
% z = result(:, 3);
% xdot = result(:, 4);
% ydot = result(:, 5);
% zdot = result(:, 6);

x = fliplr (result(:, 1));
y = fliplr (result(:, 2));
z = fliplr (result(:, 3));
xdot = fliplr (result(:, 4));
ydot = fliplr (result(:, 5));
zdot = fliplr (result(:, 6));
% 绘制结果
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

% 初始状态向量
x0 = [1000; 0; 200; 0; 0; 0]; % 初始相对距离和速度

% 时间范围和时间步长
tspan = [0 10];
dt = 0.01;

% 惯性系下的椭圆半长轴
r = 7000;

% 使用 ode45 求解 CW 方程
[t, x] = ode45(@(t, x) clohessy_wiltshire(t, r, x), tspan, x0);

% 绘制相对距离随时间的变化
figure;
plot(t, x(:, 1), 'r', t, x(:, 2), 'g', t, x(:, 3), 'b');
legend('x', 'y', 'z');
xlabel('时间');
ylabel('相对距离');
grid on;


function dxdt = clohessy_wiltshire_external(t,r, x, F_ext)
    % CW equations with external force
    mu = 3.986004418e14; % Earth's gravitational parameter (m^3/s^2)
    n = sqrt(mu / r^3); % Mean motion

    A = [0      0      0      1     0     0;
         0      0      0      0     1     0;
         0      0      0      0     0     1;
         0      0      0      0     0    2*n;
         0    -n^2     0      0     0     0;
         0      0     3*n^2  -2*n   0     0];

    B = [0  0  0;
         0  0  0;
         0  0  0;
         1  0  0;
         0  1  0;
         0  0  1];
    dxdt = A * x + B*F_ext;
end

function dxdt = clohessy_wiltshire(t,r, x)
    % CW equations with external force
    mu = 3.986004418e14; % Earth's gravitational parameter (m^3/s^2)
    n =  sqrt(mu / r^3);
    A = [0      0      0      1     0     0;
         0      0      0      0     1     0;
         0      0      0      0     0     1;
         0      0      0      0     0    2*n;
         0    -n^2     0      0     0     0;
         0      0     3*n^2  -2*n   0     0];

    dxdt = A * x ;
end