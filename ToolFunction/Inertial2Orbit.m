%% 惯�?�系到轨道系转换矩阵
function L_oi = Inertial2Orbit ( RV )

    R=RV(1:3);
    V=RV(4:6);
    if norm(R) <= 0 
        error('Satellite Postiontion norm(R) = 0 in GetRoi !'); 
    end
    
	k = -R/norm(R);
    
    H = cross(R,V); 
    j = -H/norm(H);
    j = j/norm(j);
    i = cross(j,k);
    i = i/norm(i);
    
    L_oi = [i';j';k'];
%     RV_orbit=[L_oi*RV(1:3);L_oi*RV(4:6)];
end