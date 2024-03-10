function [delta,QXx]= Inertial2Relative(R1,V1,R2,V2)%航天器2相对于航天器1的轨道根数在1的轨道系下的表示
%参见轨道力学 Howard D.Curtis 周建华P254-P256
mu=3.986005e5;%km^3/s^2
radTOdeg=180/pi;
degTOrad=pi/180;
% R1=R1*1000;
% V1=V1*1000;
% R2=R2*1000;
% V2=V2*1000;
a1=-mu*R1/(norm(R1))^3;
a2=-mu*R2/(norm(R2))^3;%两航天器的加速度

i=R1/norm(R1);%与航天器A固连的运动坐标系x轴的单位矢量i
h1=cross(R1,V1);%z轴与ha方向一致
k=h1/norm(h1);
j=cross(k,i);
    
OMG1=h1/(norm(R1))^2;%固连与航天器A上的xyz坐标系的角速度
% deltaOMG1=(-2*dot(R1,V1)/(norm(R1))^2)*OMG1;%角加速度

deltaR=R2-R1;%B相对于固连于A的运动坐标系的位置矢量和速度矢量
deltaV=(V2-V1)-cross(OMG1,deltaR);
% deltaA=a2-(a1+cross(deltaOMG1,deltaR)+cross(OMG1,cross(OMG1,deltaR))+2*cross(OMG1,deltaV));

QXx=[i';j';k'];%由惯性系XYZ到轨道系xyz
deltaR=QXx*deltaR;
deltaV=QXx*deltaV;
% deltaA=QXx*deltaA;
% deltaR=[deltaR(2);-deltaR(3);-deltaR(1)];
% deltaV=[deltaV(2);-deltaV(3);-deltaV(1)];
% deltaA=[deltaA(2);-deltaA(3);-deltaA(1)];%周建华P254-P256  到我们定义的目标星相对坐标系  【x y z】到【y -z -x】
delta=[deltaR;deltaV];%以km和km/s输出  x轴沿r0方向 y向前   而dh的轨道坐标系是x轴向前 y轴向右 z轴向下

end