%输入卫星数据
JD_startTime = [2022,1,1,0,0,0];

red = [42100 0 0.1 92 0 280];
blue = [42166 0 0.1 92 0 280];
%转化为rv
global GM_Earth
[red_r, red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
[blue_r, blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
Red_rv = [red_r; red_v];
Blue_rv = [blue_r ;blue_v];

red_hight = squared_sum(red_r);
blue_hight = squared_sum(blue_r);
%使用霍曼转移
[deltv1,deltv2,Tf] = Hm_transfer(blue_hight,red_hight);
% 计算角度

%粒子群算法

%最优控制理论

%遗传算法

%粒子群算法

%%
%二分法

for x = red_hight:blue_hight
%     x=red_hight+1%后面进行迭代,单位是km/s
    [deltv_red1,deltv_red2,Tf1] = Hm_transfer(red_hight,x);
    [deltv_red3,deltv_red4,Tf2] = Hm_transfer(x,red_hight);
    %计算推力大小
    push_value = squared_sum(deltv_red1)+squared_sum(deltv_red2)+squared_sum(deltv_red3)+squared_sum(deltv_red4);
    Red_rv = Red_rv + deltv_red1;
    
    if Tf1+Tf2<=Tf
        printf('Yes')
        for i = 0:Tf
            %Blue飘飞计算
            Blue_rv_2 = twoBodyOrbitRV(Blue_rv,1);
            if(i==Tf1)
                Red_rv = Red_rv + deltv_red2;
            end

            if(i==Tf-Tf2)
                Red_rv = Red_rv + deltv_red3;
            end
            Red_rv_2 = twoBodyOrbitRV(Red_rv,1);
            %角度计算

            %xc追踪和xt任务星rv
            [idealDegree,actualDegree] = IlluminationAngle(Red_rv_2,Blue_rv_2,JD_startTime);

            Blue_rv = Blue_rv_2;
            Red_rv = Red_rv_2;
            %计算收益
            J = J + actualDegree * i ;
        end
        %最后增益
        J_F=J + push_value;
    end
end


% 欧式距离
function y = squared_sum(x)
y =  sqrt(sum(x.^2));
end