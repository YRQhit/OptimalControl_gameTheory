function [A]=Flimit(A,B)
% FLIMIT ����������A�ĵ�n������B�ĵ�n�е�Լ��
% ʹ�÷���Ϊ result = Flimit(A,B)
% ���õĲ���A,B ����
% A = [5 7 10;6 3 2];
% B = [2 3;4 6;1 8];

[rowA,colA] = size(A);
[rowB,colB] = size(B);

if colB ~= 2 
    error('��ȷ��B������Ϊ2����BΪA��ֵ�ķ�Χ')
end 
if colA ~= rowB
    error('��ȷ��A����������B������');
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

