function [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q)


    t = Consts.t ;

    G = zeros(18 , 6) ;
    G(1 : 3 , 1 : 3) = -kalman.cbn;
    G(4 : 6 , 4 : 6) = kalman.cbn;

    F = Fai(kalman ) ;   % 状态转移矩阵


    Q = G * Q * G' ;
    Q_ = Q * Consts.t + (F * Q + (F * Q)') * Consts.t ^ 2 / 2 ;



    Z2 = kalman.vel' ;
    HH2 = [zeros(3,3) ,  diag([1 , 1 , 1]) ,  zeros(3 , 12)];      % ZUPT
    R2 = diag( [1 , 1 , 1] * 0.00001) ;

    X_expect = (eye(18) + F * t) * X_filter;    % kalman方程1
    P_expect = (eye(18) + F * t) * P_filter * (eye(18) + F * t)' + Q_ ;   % kalman方程2

    K2 = P_expect * HH2' * (pinv(HH2 * P_expect * HH2' + R2)) ;               % kalman方程3
    X_filter = X_expect + K2 * (Z2 - HH2 * X_expect) ;                     % kalman方程4
    P_filter = (eye(18) - K2 * HH2) * P_expect * (eye(18) - K2 * HH2)' + K2 * R2 * K2' ;     % kalman方程5
end