function T_f = TimeSolve_1(T_0 , coe)
global GM_Earth;
[r_t_0 , v_t_0] = Orbit_Element_2_State_rv(coe , GM_Earth);
x_t_0 = [r_t_0 ; v_t_0];

yy = OrbitPrediction(x_t_0 , T_0 , 120,[1 1],'RK7');
r_t_0 = yy (1 : 3);
v_t_0 = yy (4 : 6);

x_t_0 = [r_t_0 ; v_t_0];
k = 1;
time_step = 10;

while(1)
    
    yy1 = OrbitPrediction(x_t_0 , time_step , 5,[1 1],'RK7');
    r_t_f = yy1 (1 : 3);
    v_t_f = yy1 (4 : 6);
    
    z_1 = r_t_0(3);
    z_2 = r_t_f(3);
    
    if z_1 * z_2 <= 0
        
        t_0_temp = 0;
        t_f_temp = time_step;
        z_1_temp = z_1;
        z_2_temp = z_2;
        
        for j = 1 : 7
            yy = OrbitPrediction(x_t_0  , (t_f_temp - t_0_temp) / 2 , 0.05,[1 1],'RK7');
            r_f_temp = yy (1 : 3);
            
            
            if z_1_temp * r_f_temp(3) <=0
               t_f_temp =  (t_0_temp + t_f_temp) / 2;               
            else
                t_0_temp = (t_0_temp + t_f_temp) / 2;
                z_1_temp = r_f_temp(3);
                x_t_0 = yy;
            end
        end
        
%         T_f = T_0 + time_step * k + (t_0_temp + t_f_temp) / 2;
        T_f = T_0 + time_step * (k-1) + t_f_temp;
        break;
    end
    
    k = k + 1;
    
    x_t_0 = [r_t_f ; v_t_f];
    r_t_0 = r_t_f;
    
end
end