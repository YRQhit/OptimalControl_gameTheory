function [T_piao,T_chonghe,lonlat_c_chonghe,dv1,dv2,Tw,Flag_Tpavai] = xingxiadian(rvc,rvt,coe_c,coe_t,dr,lonlat,date)
% XINGXIADIAN ���ڼ���Ʈ�ɶ೤ʱ���ʼ�ӷɹ����������������µ㾭γ��Ҫ��
% ִ�иù���ʱ����Ϊ׷���Ǻ�Ŀ������ͬһ�����ֻ�����һ�������
% ����˼·����
%% ����2
dfai = AmendDeg(coe_t(6)+coe_t(5)-(coe_c(6)+coe_c(5)));
dfai = dfai*pi/180;
signdfai = sign(dfai);

%% �г�һȦ�ڵľ�γ�ȹ�ϵ���ձ�
[lonlattable,tardata,T_t] = lonlattablegen(coe_t,rvt,date);                 % T_t Ŀ����
node1 = 1; 
%% �����Ŀ�꾭γ������ľ�γ��
deltalonlattable = lonlat - lonlattable(1:2,:);
for i = 1 : size(deltalonlattable,2)
    deltavaluetable(i) = norm(deltalonlattable(:,i));
end
[~,index] = min(deltavaluetable);
%% ��ôӵ�ǰʱ���𣬵�Ŀ��λ������Ҫ��ʱ��T_need
if index~= size(deltalonlattable,2)
    T_need = 60*(index -1);
else
    T_need = T_tarcircle;
end
node2 =1;
% ��Ӧ��λ���ٶȺ���Ҫ��
rvt_final = tardata(:,index);
coe_t_final = State_rv_2_Orbit_Element(rvt_final(1:3),rvt_final(4:6));
%% ���˺�STK����Ӧ��  20210621-21:04
node2 =1 ; % ���˽ڵ㣬�����һȦ�����ĸ���׼ȷ��γ���Լ�Ŀ���ǵ�����Ŀ�꾭γ������ĵ�����Ҫ����ʱ��

%% ���ˣ����Ŀ���ǵ�Ŀ��λ������Ҫ��ʱ�䣬���ٶ��������
%% ���ݳ�ʼ�ĽǶȹ�ϵȷ���ǽ��컹������
if dfai > 0
    rlve = norm(rvt(1:3))-dr;
else
    rlve = norm(rvt(1:3))+dr;
end

[delta_v1, delta_v2, ~, Tw]=Hm_Iteration(rvc, rlve, date);
rvc_ellip = rvc + [0;0;0;rvc(4:6)/norm(rvc(4:6))*delta_v1];                  % ����delta_v1�ٶ���������ת�ƹ��
[rvc_beforelve,ChaserData] = OrbitPrediction(rvc_ellip ,Tw ,60,[1,1],'RK7');             % ����ʱ��Tw��ת�ƹ������λ���ٶ�

%% ׷���ǽ����ӷɹ��ʱ�̣�׷���Ǻ�Ŀ���ǵ���Ҫ�غ�λ���ٶ�
rvc_lve = rvc_beforelve+[0;0;0;delta_v2];
coe_c_lve = State_rv_2_Orbit_Element(rvc_lve(1:3),rvc_lve(4:6));
[rvt_lve,TargetData] = OrbitPrediction(rvt ,Tw ,60);
coe_t_lve = State_rv_2_Orbit_Element(rvt_lve(1:3),rvt_lve(4:6));

% �˴�����Ŀ���Ǻ�׷���ǵ�ǰ���ϵδ�ı�
dfai2 = AmendDeg(coe_t_lve(6)+coe_t_lve(5)-(coe_c_lve(6)+coe_c_lve(5)));
signdfai2 = sign(dfai2);
dfai2 = signdfai*dfai2*pi/180;      % �����ǵ�dfai2�������

% ���׷���Ǻ�Ŀ�����Թ��ת�ƿ�ʼʱ�̵���������Ϊ�ģ����ǶԵ����Ž�Ϊ0ʱ������Ҫ��ʱ��
omega_fast = omegacal(min(coe_c_lve(1),coe_t_lve(1)));
omega_slow = omegacal(max(coe_c_lve(1),coe_t_lve(1)));                                 % **�¼�
Twait = dfai2 / (omega_fast - omega_slow);
T_chonghe = Twait + Tw;
[T_piao,Flag_Tpavai] = tp_guarantee(T_need,T_chonghe,T_t);                             % ***

%% ������ʵ��dv1,dv2���غ�ʱ ׷�������µ�ľ�γ��
if Twait >0 && Flag_Tpavai == 1                                                        % ****
    % Ʈ��һ��ʱ���ת�ƿ�ʼʱ�̵�׷����λ���ٶ�rvc_transstart
    rvc_transstart = OrbitPrediction(rvc,T_piao,60);    
    [dv1, dv2, ~, Tw]=Hm_Iteration(rvc_transstart, rlve, date);
    % ʵ�ʽ���ת�ƹ����׷����λ���ٶ�
    rvc_ellip_act = rvc_transstart + [0;0;0;rvc_transstart(4:6)/norm(rvc_transstart(4:6))*dv1];                  % ����delta_v1�ٶ���������ת�ƹ��
    [rvc_zhuanyi_act,ChaserData] = OrbitPrediction(rvc_ellip_act ,Tw ,60,[1,1],'RK7');
    %% ʵ�ʽ����ӷɹ��ʱ�̵�׷����λ���ٶ�rvc_lve_actual
    rvc_lve_actual = rvc_zhuanyi_act + [0;0;0;dv2];
    % ʵ���غϵ�λ���ٶ�
    rvc_chonghe_act = OrbitPrediction(rvc_lve_actual ,Twait ,60,[1,1],'RK7');
    
    %% ��������������ת����ɺ�׷���Ǻ�Ŀ���ǵ�ǰ���ϵû�б仯���µ�׷�������µ�����վ�γ��
    [lonlat_c,E2] = rv2lonlat(rvc_chonghe_act,AddTime(date,T_need));
    [~,lat,~] = Geodetic(E2*rvc_chonghe_act(1:3));
    lonlat_c_chonghe = [lonlat_c(1);lat];
end                                                                                    % *********
node3 = 1;

if signdfai2*signdfai <0     % ��˵���ڻ���ת�ƹ���������������ص��ĵط�
    for i = 1: size(TargetData,2)
        crs(i) = norm(cross(TargetData(1:3,i),ChaserData(1:3,i)));
    end
    [~,index] = min(crs);
    T_chonghe = 60*(index-1);
    [T_piao,Flag_Tpavai] = tp_guarantee(T_need,T_chonghe,T_t);
    if Flag_Tpavai == 1                                                                 % ********
        %% ���غ�ʱ���rvc
        % ����ת�ƿ�ʼʱ�̵�׷����λ���ٶ�rvc_transstart
        rvc_transstart = OrbitPrediction(rvc,T_piao,60,[1,1],'RK7');
        [dv1, dv2, ~, Tw]=Hm_Iteration(rvc_transstart, rlve, date);
        % ʵ�ʽ���ת�ƹ����׷����λ���ٶ�
        rvc_ellip_act = rvc_transstart + [0;0;0;rvc_transstart(4:6)/norm(rvc_transstart(4:6))*dv1];
        %% �����غϵ�ʱ���׷����λ���ٶ�rvc_chonghe
        rvc_chonghe = OrbitPrediction(rvc_ellip_act,T_chonghe,60,[1,1],'RK7');
        [lonlat_c,E2] = rv2lonlat(rvc_chonghe,AddTime(date,T_need));
        [~,lat,~] = Geodetic(E2*rvc_chonghe(1:3));
        lonlat_c_chonghe = [lonlat_c(1);lat];
    end                                                                                 % *******
end

if Flag_Tpavai == 0
    T_chonghe = -1;
    lonlat_c_chonghe = NaN;
    dv1 = 0;
    dv2 = zeros(3,1);
end


%% 0ʹ�ô�Χת��������µ�ִ﹤��
% rlve = norm(rvt(1:3))-dr;
% TA = tae(coe_t_final);
% n =0;
% [control, result] = dafanweizhuanyi(coe_c, [rlve;0;coe_t_final(3:4);0;TA],T_total+ n *T_tarcircle, 0, 'HPOP');
% while result.sign == 0
%     n = n+1;
%     [control, result] = dafanweizhuanyi(coe_c, [rlve;0;coe_t_final(3:6)],T_total+ n *T_tarcircle, 0, 'HPOP');
% end
% % [control, result] = dijin(coe_c, [rlve;0;coe_t_final(3:4);0;TA], 0, 'HPOP');
% node_all = 1;
end
function [T_piao,Flag_avai] = tp_guarantee(T_need,T_chonghe,T_t)
% ���ڱ�֤T_piao �ĺ�����
% T_need ��Ŀ����Ʈ��Ŀ��λ������Ҫ��ʱ��
% T_chonghe ��׷����׷��Ŀ��������Ҫ��ʱ��
% T_t �Ƕ�������£�Ŀ���ǵĹ������
% �����
% T_piao ��Ʈ��ʱ��
% Flag_avai ��Tpiao���ñ�־λ�����Ϊ1 ����ã�Ϊ0 ����Ҫ��ִ�����T_piao �Ļ������ٶȼ���

if T_need >= T_chonghe
    T_piao = T_need - T_chonghe;
    Flag_avai = 1;
else
    T_piao = T_need + 0.5 * T_t;
    Flag_avai = 0;
end
end





