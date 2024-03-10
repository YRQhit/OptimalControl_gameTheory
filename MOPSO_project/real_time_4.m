%实时模拟
global flag;
flag =1;
global red_max_push;
global blue_max_push;
global Blue_rv
global Red_rv
global red_position;
global blue_position;
global decide_time;
decide_time = 60*60;
global predict_time;
predict_time = 60*60;
red_max_push = 0.5;
blue_max_push  = 0.5;
global JD_startTime
global JD_endTime
JD_startTime = datetime([2022,1,1,0,0,0]);
JD_endTime = datetime([2022,1,2,0,0,0]);
red = [42100 0 0.1 92 0 280];
blue = [42100 0 1 92 0 280];
[Red_r,Red_v] = Orbit_Element_2_State_rv(red, GM_Earth);
[Blue_r,Blue_v] = Orbit_Element_2_State_rv(blue, GM_Earth);
Red_rv=[Red_r;Red_v];
Blue_rv=[Blue_r;Blue_v];
red_position = [Red_rv];
blue_position = [Blue_rv];
% escape_distance = 100;s
delta_velocity = MOPSO_nashi(decide_time)
delta_velocity_blue = [0 0 0];
delta_velocity_red = [0 0 0];
delta_velocity_blue = delta_velocity(4:6)
delta_velocity_red = delta_velocity(1:3);
% red_track = [];
% blue_track = [];
% %测试双方博弈的代码
% %假设蓝方希望接近红方，红方距离比较近的时候采取规避或者其他的动作
% while(JD_startTime<JD_endTime)
%     disp("红蓝相聚距离")
%     disp(norm(Red_rv(1:3)-Blue_rv(1:3)))
%     delta_velocity_blue = [0 0 0];
%     delta_velocity_red = [0 0 0];
%     if(norm(Red_rv(1:3)-Blue_rv(1:3))>100)
%         disp("抵近中")
%         decide_time = 60*60;
%         flag =1;
%         delta_velocity_blue = MOPSO_blue(decide_time)
%         disp("抵近计算完成")
%     end
%     if(norm(Red_rv(1:3)-Blue_rv(1:3))<10)
%         disp("规避中")
%         decide_time = 60*10;
%         flag =0;
%         delta_velocity_red = MOPSO_red_escape(decide_time)
%         delta_velocity_blue = MOPSO_blue_escape(decide_time)
%         disp("规避计算完成")
%     end
%     if(norm(Red_rv(1:3)-Blue_rv(1:3))>10&&norm(Red_rv(1:3)-Blue_rv(1:3))<100)
%         disp("观测中")
%         decide_time = 60*20;
%         flag =1;
%         delta_velocity_blue = MOPSO_blue_angle(decide_time)
%         delta_velocity_red = MOPSO_red_escape(decide_time)
%         disp("观测计算完成")
%     end
%     red_rv = Red_rv;
%     blue_rv = Blue_rv;
%     disp("红方转移速度")
%     disp(delta_velocity_red)
%     disp("蓝方转移速度")
%     disp(delta_velocity_blue)
%     
%     blue_rv(4:6) =  blue_rv(4:6) + delta_velocity_blue';
%     red_rv(4:6) =  red_rv(4:6) + delta_velocity_red';
% 
%     startDateTime  = JD_startTime
%     endDateTime = JD_startTime+seconds(decide_time)
%     dt = startDateTime;
%     while dt<endDateTime
%         red_rv_2 = twoBodyOrbitRV(red_rv,1);
%         blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
%         blue_rv = blue_rv_2;
%         red_rv = red_rv_2;
%         red_track = [red_track, red_rv];
%         blue_track = [blue_track,blue_rv ];
%         JD_startTime = dt;
%         if(norm(blue_rv(1:3)-red_rv(1:3))<10)      
%             disp("开始紧急规避")
%             disp("紧急规避距离")
%             disp(norm(blue_rv(1:3)-red_rv(1:3)))
%             Red_rv = red_rv;
%             Blue_rv = blue_rv;
%             guibitime = 60;
%             delta_velocity_blue = MOPSO_blue_escape(guibitime)
%             delta_velocity_red = MOPSO_red_escape(guibitime)
%             red_rv(4:6) =  red_rv(4:6) + delta_velocity_red';
%             blue_rv(4:6) =  blue_rv(4:6) + delta_velocity_blue';
% %             red_point = [red_point ,red_rv];
% %             blue_point = [blue_point , blue_rv];
%             blue_rv_2 = twoBodyOrbitRV(blue_rv,guibitime);
%             red_rv_2 = twoBodyOrbitRV(red_rv,guibitime);
%             Red_rv = red_rv_2;
%             Blue_rv = blue_rv_2;
%             red_track = [red_track, red_rv_2];
%             blue_track = [blue_track,blue_rv_2];
%             JD_startTime = JD_startTime + seconds(guibitime);
%          
% %             scatter3(red_rv(1),red_rv(2),red_rv(3))
% %             hold on
% %             scatter3(blue_rv(1),blue_rv(2),blue_rv(3))
%             
%         end
%         dt = dt+seconds(1);
%     end
%     
%     
%     disp("现在时间是")
%     disp(JD_startTime)
%     disp("红方位置")
%     disp(red_rv)
%     disp("蓝方位置")
%     disp(blue_rv)
%     Red_rv = red_rv;
%     Blue_rv = blue_rv;
% %     scatter3(red_rv(1),red_rv(2),red_rv(3))
% %     hold on
% %     scatter3(blue_rv(1),blue_rv(2),blue_rv(3))
% end


    red_track = [];
    blue_track = [];
    red_rv = Red_rv;
    blue_rv = Blue_rv;
for i = 0:decide_time-1
    %Blue飘飞计算
    red_rv_2 = twoBodyOrbitRV(red_rv,1);
    blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
    %角度计算 xc追踪和xt任务星rv

    blue_rv = blue_rv_2;
    red_rv = red_rv_2;
    red_track = [red_track, red_rv];
    blue_track = [blue_track,blue_rv ];
    %计算收益
end
    red_track_2 = [];
    blue_track_2 = [];
    red_rv = Red_rv;
    blue_rv = Blue_rv;
    blue_rv(4:6) =  blue_rv(4:6) + delta_velocity(4:6)';
    red_rv(4:6) = red_rv(4:6)+delta_velocity(1:3)';
for i = 0:decide_time-1
    %Blue飘飞计算
    red_rv_2 = twoBodyOrbitRV(red_rv,1);
    blue_rv_2 = twoBodyOrbitRV(blue_rv,1);
    %角度计算 xc追踪和xt任务星r

    blue_rv = blue_rv_2;
    red_rv = red_rv_2;
    red_track_2 = [red_track_2, red_rv];
    blue_track_2 = [blue_track_2,blue_rv ];
    %计算收益
end
scatter3(Red_r(1),Red_r(2),Red_r(3))
hold on
scatter3(Blue_r(1),Blue_r(2),Blue_r(3))
plot3(red_track(1,:),red_track(2,:),red_track(3,:),'r')
hold on
plot3(blue_track(1,:),blue_track(2,:),blue_track(3,:),'bl')
% hold on
plot3(red_track_2(1,:),red_track_2(2,:),red_track_2(3,:),'g')
hold on
plot3(blue_track_2(1,:),blue_track_2(2,:),blue_track_2(3,:),'k')

