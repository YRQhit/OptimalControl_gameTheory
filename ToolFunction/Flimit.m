function [A]=Flimit(A,B)
% FLIMIT 的作用是让A的第n列满足B的第n行的约束
% 使用方法为 result = Flimit(A,B)
% 可用的测试A,B 如下
% A = [5 7 10;6 3 2];
% B = [2 3;4 6;1 8];

[rowA,colA] = size(A);
[rowB,colB] = size(B);

if colB ~= 2 
    error('请确认B的列数为2，即B为A的值的范围')
end 
if colA ~= rowB
    error('请确保A的列数等于B的行数');
end

for j=1:colA
    for i=1:rowA
        if A(i,j)<B(j,1)
            A(i,j)=B(j,1);
        elseif A(i,j)>B(j,2)
            A(i,j)=B(j,2);
        end
    end
end

