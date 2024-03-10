function [T1,Tw1,Tw2] = extcase(r0,r1,r2,q)
% EXTCASE 用于计算极限情况，也就是r1 <= 6650km的时候设r1位6650km解T1
% 
[~,~,Tw1] = Hm_transfer(r0,r1);
[~,~,Tw2] = Hm_transfer(r1,r2);
omegat = omegacal(r0);      % 初始轨道的平角速度，可能不是这个式子，但是没大所谓
Tw = Tw1 +Tw2;
omegap = omegacal(r1);
T = (2*pi - omegap * Tw - q)/(omegat - omegap);
T1 = T - Tw;
end