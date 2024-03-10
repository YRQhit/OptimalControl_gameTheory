% 主矢量分析
% 输入：x1 - 初始位置速度     t - 时间分配     
%       deltv - 脉冲          step - 步长
function [r, time, y1, y2] = primeVectorAnalyze(x1, t, deltv, step)
n = length(t);  rv = x1;
time = [];  r = []; 
y1 = [];  y2 = [];
r0 = zeros(3,2);  temp = 0;
for i = 1:n
    [rv, chaseData] = OrbitPrediction(rv, t(i), step, [0 0], 'RK7');
    if i ~= 1
        r0(:,1) = deltv(:,i-1)/ norm(deltv(:,i-1));
    end
    
    if i~= n
        r0(:,2) = deltv(:,i) / norm(deltv(:,i));
        rv(4:6) = rv(4:6) + deltv(:,i);
    else
        r0(:,2) = [0;0;0];
    end
    
    if t(i) == 0
        continue;
    end
    
    if i == 1
        [r1,r2] = computePrimeVector(chaseData, t(i), 0,step, r0);
    else
        [r1,r2] = computePrimeVector(chaseData, t(i), sum(t(1:i-1)), step, r0);
    end
        
    for j = 1:size(r1, 2)
        r = [r norm(r1(:,j))];
        y1 = [y1 r1(:,j)];   y2 = [y2, r2(:,j)];
        
        time = [time  temp + (j-1)*step];
        if j == length(r1)
            time(end) = temp + t(i);
        end
    end
    
    temp = time(end);
end

% for i = 1:length(time)-1
%     if r(i) == r(i+1)
%         r(i) = [];
%         time(i) = [];
%     end
% end
plot(r);
end
