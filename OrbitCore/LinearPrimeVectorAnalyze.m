% 线性模型主矢量分析
% 输入：t -  时间分配     deltv - 脉冲
%       step - 步长        w - 角速度
% 输出：r - 主矢量模值       tdata - 时间序列
%       y1 - 位置协态变量      y2 - 速度协态变量
% 例：deltv = [-0.00108405168297091;-0.00184868986132668;-3.09181174240777e-09;-2.60482363478535e-05;
%             -1.60871363930139e-05;1.58791316826495e-09;-2.51056538179691e-05;-0.000285067633191353;
%             -1.73579888020342e-09;-0.00225093918440406;0.00215416273469891;2.84949725728876e-09];
%     [r, tdata, y1, y2] = LinearPrimeVectorAnalyze([0,1800,1305,1895,0],
%     deltv, 60, 0.001105128988058);
function [r, tdata, y1, y2] = LinearPrimeVectorAnalyze(t, deltv, step, w)
n = length(t);  y1 = [];  y2 = []; 
tdata = [];    Temp = 0;
for i = 1:n
    T = t(i);
    if T == 0
        continue;
    elseif i == 1 && t(1) ~= 0
        T = -T;   step = -step;
        p1 = deltv(3*i-2:3*i) / norm(deltv(3*i-2:3*i));
        p2 = [0;0;0];
    elseif i~=n
        step = abs(step);
        p1 = deltv(3*i-5:3*i-3) / norm(deltv(3*i-5:3*i-3));
        p2 = deltv(3*i-2:3*i) / norm(deltv(3*i-2:3*i));
    else
        p1 = deltv(3*i-5:3*i-3) / norm(deltv(3*i-5:3*i-3));
        p2 = [0;0;0];
    end
    
    [~, Qrv, ~, Qvv] = CWStateTransitionMatrix(w, -T);  
    A = Qrv'; B = Qvv';
    d = A \ (p2 - B * p1);
    
    num = floor(T/step);
    if rem(T, step) ~= 0
        num = num + 1;
    end
    
    for j = 1 : num
        if ~isempty(y1) && j == 1
            continue;
        end
        
        time = (j-1) * step;
        if j == num
            time = T;
        end
        
        [Qrr, Qrv, Qvr, Qvv] = CWStateTransitionMatrix(w, -time); 
         
        y1 = [y1 Qrr'*d+Qvr'*p1];
        y2 = [y2 Qrv'*d+Qvv'*p1];
        
        if i == 1
            tdata =[tdata t(i)+time];
        else
            tdata = [tdata sum(t(1:i-1)) + abs(time)];
        end
    end

    if i == 1
        y1 = fliplr(y1);
        y2 = fliplr(y2);
        tdata = fliplr(tdata);
    end
end

r = zeros(size(y1,2), 1);
for i = 1:size(y1,2)
    r(i) = norm(y2(:,i));
end
end

