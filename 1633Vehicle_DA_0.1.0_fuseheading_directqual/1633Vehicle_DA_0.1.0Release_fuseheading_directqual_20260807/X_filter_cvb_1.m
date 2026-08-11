function [x_filter_cvb , p_filter_cvb] = X_filter_cvb_1(x_filter_cvb , p_filter_cvb , cbn , vodo , dt , cvb)

ds = vodo * dt  ;
M = [0 , -ds ; 0 , 0 ; ds , 0] ;
N = cbn * cvb * [0 , vodo , 0]' * dt ;
N1 = [0 , -N(3) , N(2) ; N(3) , 0 , -N(1) ; -N(2) , N(1) , 0];

F = zeros(9 , 9) ;
F(1 : 3 , 4 : 5) = -cbn * cvb * M ;
F(1 : 3 , 6 : 8) = N1 ;
F(1 : 3 , 9) = N ;

Q_ = diag([[0.01  , 0.01  , 0.01]  , [0.001 , 0.001] ,  [0.001 , 0.001 , 0.001] , 0.0001]) ^ 2 ;

X_expect = (eye(9) + F ) * x_filter_cvb;
P_expect = (eye(9) + F) * p_filter_cvb * (eye(9) + F )' + Q_ ;
x_filter_cvb = X_expect ;
p_filter_cvb = P_expect ;























end