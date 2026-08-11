function [v_odo , vodo_behind] = Odometer_speed(imu, k0001 , k0002 , cbn , cvb) 
       
            %   后轮里程计
            vodo_behind = k0001 * (imu(10) + imu(11)) / 2;
            
            if imu(8) == 2 || imu(9) == 2
                vodo_behind = - vodo_behind ;
            end
         
            v_odo = cbn * cvb * [ 0 , vodo_behind , 0 ]' ;   % 只用后轮，前轮不能用
end