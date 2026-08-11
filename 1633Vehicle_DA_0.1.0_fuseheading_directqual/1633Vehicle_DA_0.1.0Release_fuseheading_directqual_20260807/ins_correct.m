function [q, cbn, vel, pos, x_filter, p_filter] = ins_correct(a , gps) 

    q0 = cos(a(3)/2)*cos(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*sin(a(1)/2)*sin(a(2)/2);
    q1 = cos(a(3)/2)*sin(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*cos(a(1)/2)*sin(a(2)/2);
    q2 = cos(a(3)/2)*cos(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*sin(a(1)/2)*cos(a(2)/2);
    q3 = cos(a(3)/2)*sin(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*cos(a(1)/2)*cos(a(2)/2);

    q = [q0 ; q1 ; q2 ; q3] / norm([q0 ; q1 ; q2 ; q3]);

    cbn = [q0^2+q1^2-q2^2-q3^2,     2*(q1*q2+q0*q3),       2*(q1*q3-q0*q2);
           2*(q1*q2-q0*q3),         q0^2-q1^2+q2^2-q3^2,   2*(q2*q3+q0*q1);
           2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),       q0^2-q1^2-q2^2+q3^2]';

    vel = gps(5 : 7) ;
    pos = gps(1 : 3) ;
    x_filter = zeros(18 , 1) ;
    p_filter = diag([[1 , 1 , 1] * 0.01 *  pi / 180 , [0.1 , 0.1 , 0.1] * 1, [0.1 / Consts.re , 0.1 / Consts.re , 0.1] * 1 ,...
                    [1 , 1 , 10] / 57 / 3600 * 1 , [1e-3 , 1e-3 , 1e-3 ] * 0.1, [50/57 , 10 , 30/57 ] * 1e-1])^2 ; %误差方差

end