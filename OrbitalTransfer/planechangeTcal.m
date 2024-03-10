function [T,Dir_intersection,n2_0] = planechangeTcal(rvc,rvt)
% PLANECHANGETCAL ���ڼ����ԭ����Բ���ת�Ƶ������Բ�����ʱ�̣�����xcΪ0ʱ�̣��Լ�����Ҫ��deltaV
% ������ô����Ŀ�������ú�������ת�Ƶ�����߶���ͬ��Ŀ����ƽ�档
% rvc ��׷���ǵ�λ���ٶ���Ϣ��6*1����������λkm
% rvt ��Ŀ���ǵ�λ���ٶ���Ϣ��6*1����������λkm

%%  �������������r��v
% ׷������ز���
rc = rvc(1:3);
vc = rvc(4:6);
rc_val = norm(rc);
vc_val = norm(vc);

% Ŀ������ز���
rt = rvt(1:3);
vt = rvt(4:6);

%% �ò�˼��㵥λ������,��ý���ʸ��
n1_0 = cross(rc,vc)/norm(cross(rc,vc));               % ׷��������Ĺ��ƽ�棨�����㶯��ԭ������ƽ���Ȼ���ŵ��ƶ��ı䣩
n2_0 = cross(rt,vt)/norm(cross(rt,vt));               % Ŀ��������Ĺ��ƽ�棨�����㶯��ԭ������ƽ���Ȼ���ŵ��ƶ��ı䣩

Dir_intersection = cross(n1_0,n2_0)/norm(cross(n1_0,n2_0));                  % �����������ƽ��Ľ���ʸ��

%% �����������ת��λ�ã�����˳�й������������ʱ��׷���ǵ�λ���ٶ���Ϣ
% rtrans_1 = Dir_intersection * rc_val;
% rtrans_2 = - Dir_intersection * rc_val;

%% ���rvʸ����ת��Ҫ�أ������������Ϣ������rvת��Ҫ�ض���Բ����������ǻ����죬���Բ������������

% rv_trans1 = [rtrans_1;vtrans_1];
% rv_trans2 = [rtrans_2;vtrans_2];

% coe_trans1 = State_rv_2_Orbit_Element(rtrans_1 ,vtrans_1);
% coe_trans2 = State_rv_2_Orbit_Element(rtrans_2 ,vtrans_2);

%% ���rvʸ����ֱ��ͨ���ǶȵĶ��壬���ڻ�����ýǶ�,arccos ������0��pi����ʱ���������1 Ϊ��ý��Ŀ�ת�Ƶ��Ӧ����ǲ�
cosXJ1 = dot(rc,Dir_intersection)/(norm(rc)*norm(Dir_intersection));
cosXJ2 = dot(rc,-Dir_intersection)/(norm(rc)*norm(Dir_intersection));


if cosXJ1 > 0           % ��ζ�����1 �����
    XJ1 = acos(cosXJ1);
elseif cosXJ1 < 0       % ��ζ�����1 ��һ������180�ȵĶ۽�
    XJ1 = acos(cosXJ2); % ����ζ�ţ���ý����Ǹ����������1
    Dir_intersection = -Dir_intersection;
end
% XJ2 = pi+XJ1;           % ���2 ������Ĭ�������1 ��ʱ��ͽ���ת�ƣ��������2 ��ʱ���ã���ע�͵�

% ������ٶ�
omegac = omegacal(norm(rc));

% ��������£�Ӧ����T_ideal�����ת�ƣ�������Ϊ���㶯������������0.95*T_ideal���ټ���һ�ν�����Ϊ����
T = XJ1/omegac;

end

