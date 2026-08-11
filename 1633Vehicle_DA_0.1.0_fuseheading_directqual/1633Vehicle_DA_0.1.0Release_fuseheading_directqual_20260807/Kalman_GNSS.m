function [X_filter , P_filter] = Kalman_GNSS(kalman , X_filter , P_filter , Q , R) 

    H = kalman.pos(3);
    t = Consts.t ;

    G = zeros(18 , 6) ;
    G(1 : 3 , 1 : 3) = -kalman.cbn;
    G(4 : 6 , 4 : 6) = kalman.cbn;

    F = Fai(kalman ) ;   % 状态转移矩阵


    Q = G * Q * G' ;
    Q_ = Q * Consts.t + (F * Q + (F * Q)') * Consts.t ^ 2 / 2 ;


    a_3 = kalman.att(3) - kalman.z(4) ;
    if  a_3 > 210 /180 * pi
        a_3 = a_3 - 2 * pi ;
    elseif a_3 < -210 /180 * pi
        a_3 = a_3 + 2 * pi ;          %航向观测量处理
    end

    if kalman.flag1 == 0
        kalman.z(5 : 7) = [0 , 0 , 0] ;
    end

    Z=[  kalman.vel(1) - kalman.z(5) ;     kalman.vel(2) - kalman.z(6) ;        kalman.vel(3) - kalman.z(7) ; ...
        (kalman.pos(1) - kalman.z(1)) * kalman.rm ;       (kalman.pos(2) - kalman.z(2)) * (kalman.rn * cos(kalman.pos(1))) ;...
         H - kalman.z(3) ; a_3  ];    %观测量


    if kalman.flag_heading_fusion == 1 && kalman.goe(9) == 3
        H_heading = 1 ;
    else
        H_heading = 0 ;
        Z(7) = 0 ;
    end

    HH = [zeros(6,3) ,      diag([1 , 1 , 1 , 1 * kalman.rm , 1 * (kalman.rn * cos(kalman.pos(1))) , 1]) ,      zeros(6 , 9);...
        [0 , 0 , H_heading] ,      zeros(1, 15)];      %

    % if kalman.yaw_correct == 0 
    %     Z(7) = 0 ;
    % end

    if kalman.goe12(2) == 1 && kalman.goe(8) > 4 / kalman.k1 || kalman.goe12(2) == 2 && kalman.goe(8) > 1.5 
        Z(6) = 0 ;
    end

    X_expect = (eye(18) + F * t) * X_filter;    % kalman方程1

 %%%%%%%%%%%%%%%%%%%%%%%%%抗差kalman%%%%%%%%%%%%%%%%%%%%%
 if kalman.goe12(2) == 4
     u1 = 50 ;
     u2 = 100 ;
     u3 = 150 ;
     % if norm(kalman.vel) < 2
     %     u3 = 15 ;
     % end
     if (kalman.goe12(1) < 10) || (kalman.goe(8) > 0.3 || kalman.goe(8) == 0)
         u1 = 1 ;
         u2 = 1 ;
         u3 = 1 ;
     end


 else
     u1 = 50 ;
     u2 = 100 ;
     u3 = 150 ;
    %  if norm(kalman.vel) < 2
    %     u3 = 15 ;
    % end
     % if (kalman.goe12(1) < 10) ...
     %         || (kalman.goe12(2) == 2 && (kalman.goe(8) > 3 || kalman.goe(8) == 0 )) ...
     %         || (kalman.goe12(2) == 1 && (kalman.goe(8) > 3 || kalman.goe(8) == 0 )) ...
     %         || (kalman.goe12(2) == 5 && (kalman.goe(8) > 0.5|| kalman.goe(8) == 0 ))
     %     if kalman.k1 == 2
     %         u1 = 0.2 ;
     %         u2 = 0.2 ;
     %         u3 = 2 ;
     %     else
     %         u1 = 1 ;
     %         u2 = 1 ;
     %         u3 = 5 ;
     %     end
     % end
 end

 if kalman.flag1 == 0
     u1 = 10000000;
 end


    ds = Z - HH * X_expect ;

    a1 = inv(chol(R( 1 : 3 , 1 : 3))) ;

    e1 = zeros(3 , 1) ;
    r1 = zeros(1 , 3) ;
    for k = 1 : 3
        e1(k , 1) = a1(k , :) * ds(1 : 3) ;
        if abs( e1(k , 1) ) < u1
            r1(k) = 1 ;
        else
            r1(k) = u1 / abs(e1(k , 1))  ;
        end
    end
    %%
    a2 = inv(chol(R( 4 : 6 , 4 : 6))) ;
    e2 = zeros(3 , 1) ;
    r2 = zeros(1 , 3) ;

    for k = 1 : 3
        e2(k , 1) = a2(k , :) * ds(4 : 6) ;
        if abs( e2(k , 1) ) < u2
            r2(k) = 1 ;
        else
            r2(k) = u2 / abs(e2(k , 1))  ;
        end
    end


    a3 = 1 / sqrt(R(7 , 7)) ;
    e3 = zeros(1 , 1) ;
    r3 = zeros(1 , 1) ;
    for k = 1 : 1
        e3(k , 1) = a3(k , :) * ds(7) ;
        if abs( e3(k , 1) ) < u3
            r3(k) = 1 ;
        else
            r3(k) = u3 / abs(e3(k , 1))  ;
        end
    end

    T = diag([r1 , r2 , r3] ) ;
    R = chol(R)' /(T) * chol(R) ;


    P_expect = (eye(18) + F*t) * P_filter * (eye(18) + F * t)' + Q_ ;   % kalman方程2
    K = P_expect * HH' * (pinv(HH * P_expect * HH' + R)) ;               % kalman方程3



    X_filter = X_expect + K * (Z - HH * X_expect) ;                     % kalman方程4
    P_filter = (eye(18) - K * HH) * P_expect * (eye(18) - K * HH)' + K * R * K' ;     % kalman方程5



end