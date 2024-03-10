function [delta,QXx]= Inertial2Relative(R1,V1,R2,V2)%������2����ں�����1�Ĺ��������1�Ĺ��ϵ�µı�ʾ
%�μ������ѧ Howard D.Curtis �ܽ���P254-P256
mu=3.986005e5;%km^3/s^2
radTOdeg=180/pi;
degTOrad=pi/180;
% R1=R1*1000;
% V1=V1*1000;
% R2=R2*1000;
% V2=V2*1000;
a1=-mu*R1/(norm(R1))^3;
a2=-mu*R2/(norm(R2))^3;%���������ļ��ٶ�

i=R1/norm(R1);%�뺽����A�������˶�����ϵx��ĵ�λʸ��i
h1=cross(R1,V1);%z����ha����һ��
k=h1/norm(h1);
j=cross(k,i);
    
OMG1=h1/(norm(R1))^2;%�����뺽����A�ϵ�xyz����ϵ�Ľ��ٶ�
% deltaOMG1=(-2*dot(R1,V1)/(norm(R1))^2)*OMG1;%�Ǽ��ٶ�

deltaR=R2-R1;%B����ڹ�����A���˶�����ϵ��λ��ʸ�����ٶ�ʸ��
deltaV=(V2-V1)-cross(OMG1,deltaR);
% deltaA=a2-(a1+cross(deltaOMG1,deltaR)+cross(OMG1,cross(OMG1,deltaR))+2*cross(OMG1,deltaV));

QXx=[i';j';k'];%�ɹ���ϵXYZ�����ϵxyz
deltaR=QXx*deltaR;
deltaV=QXx*deltaV;
% deltaA=QXx*deltaA;
% deltaR=[deltaR(2);-deltaR(3);-deltaR(1)];
% deltaV=[deltaV(2);-deltaV(3);-deltaV(1)];
% deltaA=[deltaA(2);-deltaA(3);-deltaA(1)];%�ܽ���P254-P256  �����Ƕ����Ŀ�����������ϵ  ��x y z������y -z -x��
delta=[deltaR;deltaV];%��km��km/s���  x����r0���� y��ǰ   ��dh�Ĺ������ϵ��x����ǰ y������ z������

end