%% ���Ĺ���ϵ(ICRS)�����Ĺ���ϵ(ITRS)������ת������
% ���룺Mjd - ������(UTC)   �����E - ת������
function E = ICRS2ITRS(Mjd)
UTC = Mjd;  TT = UTC + 66.184 / 86400;
NPB = iauPnm06a(2400000.5, TT);
gast = iauGst06(2400000.5, UTC, 2400000.5, TT, NPB);
Theta = iauRz(gast, eye(3));
sp = iauSp00(2400000.5, TT);
Pi = iauPom00(0, 0, sp);
E = Pi * Theta * NPB;
end

