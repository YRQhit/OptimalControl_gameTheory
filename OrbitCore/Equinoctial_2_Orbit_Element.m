% 春分点根数转轨道六根
% 例：coe = Equinoctial_2_Orbit_Element([6678.14;0.000173648177666930;0.000984807753012208;0.0441010189967332;0.250109307262788;29.9608349837944])
function coe = Equinoctial_2_Orbit_Element(equ)
a = equ(1);
e = sqrt(equ(2)^2 + equ(3)^2);
i = 2*atand(sqrt(equ(4)^2 + equ(5)^2));
raan = atand(equ(4)/equ(5));
ap = atand(equ(2)/equ(3)) - atand(equ(4)/equ(5));
f = AmendDeg(mean2True(e,equ(6) - atand(equ(2)/equ(3))),'0 - 360');
coe = [a e i raan ap f]';
end

