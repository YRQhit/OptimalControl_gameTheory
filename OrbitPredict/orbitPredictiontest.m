% x0 = [6678.137;0;0;0;6.789530;3.686410];         % 正推
% x0 = [5596.645793;-3201.967425;-1738.526464;4.215065;5.689999;3.089418];        % 倒推二体
x0 = [42166;0;0;0;3.07459;0];
startTime = [2019 1 1 0 0 0];
ParamDefine;
tic
% x1 = OrbitPrediction(x0,86400,60,[1 1],'RK7');
orbitModel = 'HPOP';   % deg = 21;
[x2,data] = OrbitPrediction(x0,86400,60,[1 1 1],'RK7',startTime);
% norm(x2-x1)
toc