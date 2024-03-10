% CW相对运动方程的状态转移矩阵
% 输入：n - 轨道角速度             t - 时间
function [Qrr, Qrv, Qvr, Qvv] = CWStateTransitionMatrix(n, t)
Qrr = [4 - 3 * cos(n*t)             0               0;
       6 * (sin(n*t) - n*t)         1               0;
       0                            0           cos(n*t)];
   
Qrv = [sin(n*t) / n              2 * (1 - cos(n*t)) / n            0;
       2 * (cos(n*t) - 1) / n    (4 * sin(n*t) - 3 *n*t) / n       0;
       0                         0                             sin(n*t) / n];

Qvr = [3 * n * sin(n*t)            0            0;
       6 * n * (cos(n*t) - 1)      0            0;
       0                           0            -n * sin(n*t)];
   
Qvv = [cos(n*t)           2 * sin(n*t)           0;
       -2 * sin(n*t)      4 * cos(n*t) - 3       0;
       0                  0                    cos(n*t)];
end


