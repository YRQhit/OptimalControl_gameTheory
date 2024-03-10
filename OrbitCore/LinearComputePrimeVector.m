% 线性模型计算主矢量
% 输入：t -  时间分配     deltv - 脉冲
%       time - 时间       w - 角速度
% 输出：r - 主矢量模值    y1 - 位置协态变量      y2 - 速度协态变量
% 例：deltv = [-0.00108405168297091;-0.00184868986132668;-3.09181174240777e-09;-2.60482363478535e-05;
%             -1.60871363930139e-05;1.58791316826495e-09;-2.51056538179691e-05;-0.000285067633191353;
%             -1.73579888020342e-09;-0.00225093918440406;0.00215416273469891;2.84949725728876e-09];
%     [r, y1, y2] = LinearComputePrimeVector([0,1800,1305,1895,0],
%     deltv, 100.5, 0.001105128988058);
function [r, y1, y2] = LinearComputePrimeVector(t, deltv, time, w)
n = length(t); 
Total = 0;
if time - sum(t) >0
    time = sum(t);
end

for i = 1:n
    if t(i) == 0
        continue;
    end
    
    Total = Total + t(i);  T = t(i);
    if time <= Total
        if i == 1
            p1 = deltv(3*i-2:3*i) / norm(deltv(3*i-2:3*i));
            p2 = [0;0;0];
            time = time - t(i);   T = -T;
        elseif i~=n
            p1 = deltv(3*i-5:3*i-3) / norm(deltv(3*i-5:3*i-3));
            p2 = deltv(3*i-2:3*i) / norm(deltv(3*i-2:3*i));
            time = time - sum(t(1:i-1));
        else
            p1 = deltv(3*i-5:3*i-3) / norm(deltv(3*i-5:3*i-3));
            p2 = [0;0;0];
            time = time - sum(t(1:i-1));
        end
        
        [~, Qrv, ~, Qvv] = CWStateTransitionMatrix(w, -T);  
        A = Qrv'; B = Qvv';
        d = A \ (p2 - B * p1);
        break;
    end
end

[Qrr, Qrv, Qvr, Qvv] = CWStateTransitionMatrix(w, -time); 
y1 = Qrr'*d+Qvr'*p1;
y2 = Qrv'*d+Qvv'*p1;
r = norm(y2);
end

