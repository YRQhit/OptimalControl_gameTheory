function xmin = binarySearchMin(f, a, b, tol)
% f: 目标函数
% a, b: 搜索区间
% tol: 容差

while abs(b - a) > tol
    % 计算中点
    x1 = (a + b) / 2;
    x2 = (x1 + b) / 2;
    
    % 计算函数值
    f1 = f(x1);
    f2 = f(x2);
    
    % 判断最小值所在的区间
    if f1 < f2
        b = x2;
    else
        a = x1;
    end
end

% 返回最小值所在的点
xmin = (a + b) / 2;
end
