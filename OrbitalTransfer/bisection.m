function [flag,x0] = bisection(func, a, b, tol )
% Bisection method for finding function root.
% [a,b]为隔根区间

fa = func(a);
fb = func(b);


if fa * fb > 0
    %error('无效的隔根区间')
    x0=0;
    flag=1;
    return
end

maxItr = ceil((log(b-a)-log(tol)) / log(2.0));

for i=1:maxItr
    c = (a + b) / 2.0;
    fc = func(c);
    
    if abs(fc) < eps
        x0 = c;
        flag=0;
        return
    end
    
    if fb * fc > 0
        b = c;
        fb = fc;
    else
        a = c;
        fa = fc;
    end
    
    if (b-a) < tol
        x0 = (a+b)/2;
        flag=0;
        return
    end
end
flag=0;
%fprintf('No solution for the specified tolerance!')
x0 = c;

