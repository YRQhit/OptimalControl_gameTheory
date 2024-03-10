%位置速度向轨道根数的转换函数
function coe = RV2COE(X)
%--------------------------------------------------------------------------
%输入
%--------------------------------------------------------------------------
%X              (6,1)                   位置速度向量
%--------------------------------------------------------------------------
%输出
%--------------------------------------------------------------------------
%coe            (6,1)                   轨道根数，
%--------------------------------------------------------------------------

Mu = 398600.436;       %开普勒常数km^3/s^2
coe=zeros(6,1);
%提示信息显示控制标志
r_v=X(1:3);
v_v=X(4:6);
d_f = false;

%计算r、v赋值
r = norm(r_v);
v = norm(v_v);

%计算半长轴
coe(1) = Mu * r / (2.0 * Mu - r * v * v);

%求指向近地点的偏心率矢量
e_v = (v * v / Mu - 1.0 / r) .* r_v - dot(r_v,v_v) / Mu .* v_v;
e = norm(e_v);

%计算偏心率
coe(2) = norm(e_v);

%计算轨道角动量矢量
h_v = cross(r_v,v_v);
h = norm(h_v); % 模值

%计算轨道倾角
k_v = [0.0;0.0;1.0]; % z轴单位矢量
%acos的主值区间是[0,pi]所以不需要角度正则化
coe(3) = acos(dot(k_v,h_v) / h); % k_v的模值为 1

%计算升节点矢量
n_v = cross(k_v,h_v);
n = norm(n_v); % 模值

%计算升节点赤经
i_v = [1.0;0.0;0.0]; % x轴单位矢量
if abs(coe(3)) < eps %倾角太小默认为 0
    coe(4) = 0.0; %赤道轨道该量赋值为 0
    if d_f
        disp('倾角太小默认为 0 所以升节点赤经无意义！默认赋值raan=0');
    end
else
    raan = acos(dot(i_v,n_v) / n); % i_v的模值为 1
    if n_v(2) < 0
        coe(4) = 2.0 * pi - raan;
    else
        coe(4) = raan;
    end
end

%计算近地点幅角
if coe(2) < eps %偏心率太小默认为圆轨道
    coe(5) = 0.0; %圆轨道该量赋值为 0
    if d_f
        disp('偏心率太小默认为圆轨道，近地点幅角无意义！默认赋值w=0');
    end
else    
    if abs(dot(n_v,e_v) / (n * e))<=1
        w = acos(dot(n_v,e_v) / (n * e));
    elseif dot(n_v,e_v)>0
        w = 0;
    else
        w = pi;
    end
        
    
    if e_v(3) < 0
        coe(5) = 2.0 * pi - w;
    else
        coe(5) = w;
    end
end

%计算真近点角
if coe(2) < eps %偏心率太小默认为圆轨道
%     coe.f = 0.0; %故意不定义该量以报错
%     disp('偏心率太小默认为圆轨道，真近点角无意义！默认赋值f=0');
    if d_f
        disp('偏心率太小默认为圆轨道，真近点角无意义！重新定义为纬度副角或真径角');
    end
else
    f = acos(dot(e_v,r_v) / (e * r));
    if dot(r_v,v_v) < 0
        coe(6) = 2.0 * pi - f;
    else
        coe(6) = f;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%处理特殊轨道情况

%对于椭圆、赤道轨道：升节点赤经 raan 和近地点幅角 w 都无意义
%定义近地点真经：从地心惯性系 x 轴转到指向近地点的偏心率矢量 e_v
%令升节点赤经  为 0，纬度副角为近地点真经 w_t
if coe(3) < eps %倾角太小默认为 0
    w_t = acos(dot(i_v,e_v) / e); % i_v的模值为 1
    if e_v(2) < 0
        coe(5) = 2.0 * pi - w_t;
    else
        coe(5) = w_t;
    end
    if d_f
        disp('倾角太小默认为 0 所以定义近地点真经 w_t 来代替升节点赤经 raan 和近地点幅角 w ！');
    end
end

%对于圆、倾斜轨道：近地点幅角 w 和真近点角 f 都无意义
%定义纬度副角：从升节点矢量 n_v 转到卫星当前位置 r_v
%令近地点幅角 w 为 0，真近点角等于纬度副角 u
if coe(2) < eps %偏心率太小默认为圆轨道
    u = acos(n_v'*r_v / (n * r));
    if r_v(3) < 0
        coe(6) = 2.0 * pi - u;
    else
        coe(6) = u;
    end
    if d_f
        disp('偏心率太小默认为 0 所以定义纬度副角 u 来代替近地点幅角 w 和真近点角 f ！');
    end
end

%对于圆、赤道轨道：升节点赤经 raan 和近地点幅角 w 和真近点角 f 都无意义
%定义真经：从地心惯性系 x 轴转到卫星当前位置 r_v
%令升节点赤经 raan 近地点幅角 w 均为 0，真近点角等于真经角 l_t
if coe(2) < eps %偏心率太小默认为圆轨道
    if coe(3) < eps %倾角太小默认为 0
        l_t = acos(dot(i_v,r_v) / r); % i_v的模值为 1
        if r_v(2) < 0
            coe(6) = 2.0 * pi - l_t;
        else
            coe(6) = l_t;
        end
        if d_f
            disp('偏心率和倾角均太小所以定义真经角 l_t 来代替升节点赤经 raan 近地点幅角 w 和真近点角 f ！');
        end
    end
end
coe(3) = rad2deg(coe(3));   coe(4) = rad2deg(coe(4));
coe(5) = rad2deg(coe(5));   coe(6) = rad2deg(coe(6));
%--------------------------------------------------------------------------
%版本信息
%--------------------------------------------------------------------------
%创建日期：2020-04-02
%修改日期：2020-04-02
%--------------------------------------------------------------------------