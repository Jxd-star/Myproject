function [X_filter , P_filter] = Kalman_odo(kalman , X_filter , P_filter , v_odo , Q , Lever_Arm_ODO)

t = Consts.t ;

R = diag( [1 , 1 , 1] * 0.01 ) ^ 2 ;

G = zeros(18 , 6) ;
G(1 : 3 , 1 : 3) = -kalman.cbn;
G(4 : 6 , 4 : 6) = kalman.cbn;

F = Fai(kalman ) ;   % 状态转移矩阵

Q = G * Q * G' ;
Q_ = Q * Consts.t + (F * Q + (F * Q)') * Consts.t ^ 2 / 2 ;

webb = [0 , -kalman.wibb(3) , kalman.wibb(2) ;     kalman.wibb(3) , 0 , -kalman.wibb(1) ;     -kalman.wibb(2) , kalman.wibb(1) , 0 ] ;
v_odo1 = v_odo ;
v_odo = v_odo - kalman.cvbn  * (webb * Lever_Arm_ODO) ;
Z = ([kalman.vel(1) - v_odo(1) ;       kalman.vel(2) -  v_odo(2) ;       kalman.vel(3) - v_odo(3)] );

cbn = kalman.cvbn ; 
mvkd = [-cbn(1 , 3) , cbn(1 , 2) , cbn(1 , 1) ; -cbn(2 , 3) , cbn(2 , 2) , cbn(2 , 1) ; -cbn(3 , 3) , cbn(3 , 2) , cbn(3 , 1) ] ;
v_odo_b = kalman.cbn' * v_odo1 ;
if abs(v_odo_b(2)) > 0.0001
    kalman.odo_vy = norm(v_odo) *  v_odo_b(2) / abs(v_odo_b(2)) ;
end
Mvkd =  kalman.odo_vy * mvkd ;

v_odo11 = cbn' * v_odo ;
X_expect = (eye(18) + F * t) * X_filter;
fai = X_expect(1 : 3 , 1) ;

v_odox = [0 , -v_odo(3) , v_odo(2) ;     v_odo(3) , 0 , -v_odo(1) ;     -v_odo(2) , v_odo(1) , 0 ] ;
v_odo_cross = [0 , -v_odo11(3) , v_odo11(2) ;     v_odo11(3) , 0 , -v_odo11(1) ;     -v_odo11(2) , v_odo11(1) , 0 ] ;

v_dn = v_odo + v_odox * fai + cbn * v_odo_cross * [X_filter(16) ; 0 ; X_filter(18)] + v_odo * X_filter(17) ;
v_dn_cross = [0 , -v_dn(3) , v_dn(2) ;     v_dn(3) , 0 , -v_dn(1) ;     -v_dn(2) , v_dn(1) , 0 ] ;

HH = [-v_dn_cross , eye(3) , zeros(3 , 9) , -Mvkd ] ;

u1 = 5.0 ;
if  abs(kalman.yaw_rate) > 0.005
    u1 = 5 * (abs(kalman.yaw_rate) / 0.005) ;
end

ds = Z - HH * X_expect ;

a1 = inv(chol(R( 1 : 3 , 1 : 3))) ;
e = zeros(3 , 1) ;
r1 = zeros(1 , 3) ;

for k = 1 : 3
    e(k , 1) = a1(k , :) * ds(1 : 3) ;
    if abs( e(k , 1) ) < u1
        r1(k) = 1 ;
    else
        r1(k) = u1 / abs(e(k , 1))  ;
    end
end
%%

T = diag(r1 ) ;
R = chol(R)' /(T) * chol(R) ;

P_expect = (eye(18) + F * t) * P_filter * (eye(18) + F * t)' + Q_ ;
K = P_expect * HH' * (inv(HH * P_expect * HH' + R)) ;
X_filter = X_expect + K * (Z - HH * X_expect) ;

P_filter = (eye(18) - K * HH) * P_expect * (eye(18) - K * HH)' + K * R * K' ;
P_filter = (P_filter + P_filter') / 2 ;

end