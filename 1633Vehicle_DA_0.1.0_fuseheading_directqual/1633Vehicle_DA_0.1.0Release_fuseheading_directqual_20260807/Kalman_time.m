function [X_filter , P_filter] = Kalman_time(kalman , X_filter , P_filter , Q)
    G = zeros(18 , 6) ;
    G(1 : 3 , 1 : 3) = -kalman.cbn;
    G(4 : 6 , 4 : 6) = kalman.cbn;

    F = Fai(kalman ) ;   % 状态转移矩阵

    Q = G * Q * G' ;
    Q_ = Q * Consts.t + (F * Q + (F * Q)') * Consts.t ^ 2 / 2 ;

    X_expect = (eye(18) + F * Consts.t) * X_filter ;
    P_expect = (eye(18) + F * Consts.t) * P_filter * (eye(18) + F * Consts.t)' + Q_  ;
    
    X_filter = X_expect ;
    P_filter = P_expect ;
end