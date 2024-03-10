function [T,index] = orbitalterTime(rv_current,date,Tremain)
% ORBITALTERPOINT��������ȷ��Բ����ı���
% rv_current ��׷���ǵ�ǰ�ٶ�
% date �ǵ�ǰʱ��
% Tremain ������ʣ��ʱ��
% ���
% T ���������������ʼ����ת�ƿ�ʼ��ʱ��
global lon_lan

coe_current = State_rv_2_Orbit_Element(rv_current(1:3),rv_current(4:6));
% Ĭ��û�ҵ�
index = -1;

%% ����nȦ�ľ�γ����Ϣ��Ĭ����Ȧ��
omegat = omegacal(coe_current(1));

T_1r= 2*pi/omegat;    % ������������ȷ��������Ȧ��
n = floor(Tremain / T_1r);
if n < 0 
    error('����ʣ��ʱ�����');
end
[rv_2c,tardata] = OrbitPrediction(rv_current,Tremain,60);   % rv_2c ����Ȧ���rv
[lonlattable] = lonlattablegen(tardata,date);

%% ����lonlattable�����Inchinaָʾ��־
for i = 1: size(lonlattable,2)
    inChina = LonLanDiscrimination(lonlattable(1:2,i));    % ��i�еľ�γ��
    if inChina == 1
        index =i;
        %% ��֤Ʈ��ʱ��Ϊ��
        if (60*(i -1)- 0.75*T_1r) < 0
            index = -1;
            continue;
        else
            break;
        end
        %% Դ����
        %         inChina2 = LonLanDiscrimination(lonlattable(:,i+1));
        %         if (inChina+inChina2) == 2
        %             index = i;
        %             break;
        %         end
        
    end
end
if index>0
    rv = tardata(:,index);  %�˴�Ϊ�뽻��㹲�ߵĳ�ʼ����ϵ�rv
    T = 60*(i-1)-0.75*T_1r;
end

if inChina == 0
    msgbox('����ָ��ʱ�����޷������й�','˫��Բ������Ϣ');
end
end