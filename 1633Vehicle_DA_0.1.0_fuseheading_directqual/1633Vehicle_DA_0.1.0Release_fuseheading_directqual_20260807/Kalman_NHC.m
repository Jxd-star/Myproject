function [X_filter , P_filter] = Kalman_NHC(kalman , X_filter , P_filter , Q , Lever_Arm_NHC)

    t = Consts.t ;

    G = zeros(18 , 6) ;
    G(1 : 3 , 1 : 3) = -kalman.cbn;
    G(4 : 6 , 4 : 6) = kalman.cbn;

    F = Fai(kalman ) ;   % 状态转移矩阵

    Q = G * Q * G' ;
    Q_ = Q * Consts.t + (F * Q + (F * Q)') * Consts.t ^ 2 / 2 ;

    webb = [0 , -kalman.wibb(3) , kalman.wibb(2) ;     kalman.wibb(3) , 0 , -kalman.wibb(1) ;     -kalman.wibb(2) , kalman.wibb(1) , 0 ] ;
    Lever_arm_imu2odo = kalman.cvbn_nhc'  * (webb * Lever_Arm_NHC) ;

    Z1 = kalman.cvbn_nhc' * kalman.vel' + Lever_arm_imu2odo ;
    vel_x = [0 , -kalman.vel(3) , kalman.vel(2) ; kalman.vel(3) , 0 , -kalman.vel(1) ; -kalman.vel(2) , kalman.vel(1) , 0] ;
    HH1 = [-kalman.cvbn_nhc' * vel_x , kalman.cvbn_nhc' , zeros(3 , 12)] ;

    Z = Z1([1 , 3] , :) ;
    HH = HH1([1 , 3] , :) ;
   

    R = diag( [1 , 1] * 0.0005 ) ^ 2 ;
    u1 = 10 ;

    % if abs(kalman.yaw_rate) > 0.005 
    %     u1 = 10 / (abs(kalman.yaw_rate) / 0.005) ;
    % end
    
    X_expect = (eye(18) + F * t) * X_filter;    % kalman方程1

    ds = Z - HH * X_expect ;
    a1 = inv(chol(R)) ;
    e1 = zeros(2 , 1) ;
    r1 = zeros(1 , 2) ;

    for k = 1 : 2
        e1(k , 1) = a1(k , :) * ds(1 : 2) ;
        if abs( e1(k , 1) ) < u1
            r1(k) = 1 ;
        else
            r1(k) = u1 / abs(e1(k , 1))  ;
        end
    end

    T = diag(r1) ;
    R = chol(R)' /(T) * chol(R) ;
    %%
    X_expect = (eye(18) + F * t) * X_filter;    % kalman方程1
    P_expect = (eye(18) + F * t) * P_filter * (eye(18) + F * t)' + Q_ ;   % kalman方程2
    K = P_expect * HH' * (pinv(HH * P_expect * HH' + R)) ;               % kalman方程3
    X_filter = X_expect + K * (Z - HH * X_expect) ;                     % kalman方程4
    P_filter = (eye(18) - K * HH) * P_expect * (eye(18) - K * HH)' + K * R * K' ;     % kalman方程5

end