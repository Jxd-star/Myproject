function [x_filter_cvb , p_filter_cvb] = X_filter_cvb(x_filter_cvb , p_filter_cvb , cbn , vodo , p_odo , p , dt , cvb , kalman)

ds = vodo * dt  ;
M = [0 , ds ; 0 , 0 ; -ds , 0] ;
N = cbn * cvb * [0 , vodo , 0]' * dt ;
N1 = [0 , -N(3) , N(2) ; N(3) , 0 , -N(1) ; -N(2) , N(1) , 0];

F = zeros(9 , 9) ;
F(1 : 3 , 4 : 5) = -cbn * cvb * M ;
F(1 : 3 , 6 : 8) = N1 ;
F(1 : 3 , 9) = N ;

Q_ = diag([[0.01  , 0.01  , 0.01]  , [0.001 , 0.001] ,  [0.001 , 0.001 , 0.001] , 0.0001]) ^ 2 ;
R = diag([ 0.001 , 0.001 , 0.001 ] * 0.1 )^2 ;
    

% HH = [kalman.rm , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ;
%       0 , kalman.rn * cos(kalman.p(1)) , 0 , 0 , 0 , 0 , 0 , 0 , 0 ; 
%       0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 ] ;
HH = [1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ;
      0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ; 
      0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 ] ;

% HH = F(1 : 3 , :)  + HH1 ;




D = diag([kalman.rm + p(3) , (kalman.rn + p(3)) * cos(p(1)) , 1]) ;
% Z =  (D) * p' - p_odo'  ;
Z =   p_odo' - (D) * p' ;

X_expect = (eye(9) + F ) * x_filter_cvb;


P_expect = (eye(9) + F) * p_filter_cvb * (eye(9) + F )' + Q_ ;
K = P_expect * HH' * (inv(HH * P_expect * HH' + R)) ;
x_filter_cvb = X_expect + K * (Z - HH * X_expect) ;
p_filter_cvb = (eye(9) - K * HH) * P_expect * (eye(9) - K * HH)' + K * R * K' ;























end