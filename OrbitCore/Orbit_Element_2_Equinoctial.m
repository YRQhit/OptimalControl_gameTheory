% 轨道六根数转春分点根数
% 例：equ = Orbit_Element_2_Equinoctial([6678.14;0.001;28.5;10;0;20])
function equ = Orbit_Element_2_Equinoctial(coe)
a = coe(1);
h = coe(2)*sind(coe(4)+coe(5));
k = coe(2)*cosd(coe(4)+coe(5));
p = tand(coe(3)/2)*sind(coe(4));
q = tand(coe(3)/2)*cosd(coe(4));
r = AmendDeg(true2Mean(coe(2),coe(6)),'0 - 360') + coe(5) + coe(4);
equ = [a h k p q r]';
end

