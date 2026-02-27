if mod(i,2)~=0
        if HighPreIMU_FLAG == 0 && IMU_Cali_Flag==1 %LLQ:2011-8-12
            ang_1 = d_angle(i,1:3)- err_gyr'*sampt0;
            vel_1 = d_vel(i,1:3)- err_acc'*sampt0;
        else
            ang_1 = d_angle(i,1:3);
            vel_1 = d_vel(i,1:3);
        end
    else
        if HighPreIMU_FLAG == 0 && IMU_Cali_Flag==1 %LLQ:2011-8-12
            ang_2 = d_angle(i,1:3)- err_gyr'*sampt0;
            vel_2 = d_vel(i,1:3)- err_acc'*sampt0;
        else
            ang_2 = d_angle(i,1:3);
            vel_2 = d_vel(i,1:3);
        end
        angle=ang_1+ang_2;
        velocity=vel_1+vel_2;
        wibb=angle/(2.0*sampt0);
        vel_scull_b = velocity+0.5*cross(angle,velocity)+(2.0/3.0)*(cross(ang_1,vel_2)+cross(vel_1,ang_2));
        wiee = [0 0 WIE]';
        %wien
        wien = [ WIE*cos(Lati)  0  -WIE*sin(Lati)]';
        %wenn
        temp = 1-eeee*sin(Lati)^2;
        rn = r0*(1-eeee)/temp^1.5+Alti;
        re = r0/temp^0.5+Alti;
        wenn = [Ve/re  -Vn/rn  -Ve*tan(Lati)/re ]';
        % g
        g_u = g0*(1+0.00193185138639*sin(Lati)^2)...
            /((1-0.00669437999013*sin(Lati)^2)^0.5 *(1.0 + Alti/r0)^2);
%             g  = [0 0 g_u]';
        g  = [0 0 0]';
        
        
        wien2_wenn_V = cross((2.0*wien + wenn),V);
        % Vel_update
        Vel_update = V + cbn * vel_scull_b' + 2*sampt0 *( g - wien2_wenn_V );
        %     Vel_update = V + cbn * vel_scull_b'+2*sampt0 *wien2_wenn_V; %%%%%%%%%%%
        V = Vel_update;
        Vn = V(1);  Ve = V(2);  Vd = V(3);
        % Pos_update
        Lati  = Lati +  2*sampt0*Vn/rn;
        Longi = Longi + 2*sampt0*Ve/(re*cos(Lati));
        %    Alti  = Alti0;
        Alti  = Alti - 2*sampt0*Vd;
        llh   = [Lati Longi Alti];
        h=llh(3);
        %wien
        wien = [ WIE*cos(Lati)  0  -WIE*sin(Lati)]';
        %wenn
        temp = 1-eeee*sin(Lati)^2;
        rn = r0*(1-eeee)/temp^1.5 + Alti;
        re = r0/temp^0.5 + Alti;
        wenn = [Ve/re  -Vn/rn  -Ve*tan(Lati)/re]';
        winn = wien + wenn;
        %winb
        winb = cnb * winn;
        %wnbb
        ang_1 = ang_1-winb'*sampt0;
        ang_2 = ang_2-winb'*sampt0;
        TurnVector = ang_1+ang_2 + (2.0/3.0)*cross(ang_1,ang_2);
        wnbb=TurnVector'/(2*sampt0);
