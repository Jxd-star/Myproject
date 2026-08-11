function cbn = a2m(a0)
        a = a0 ;
        q0 = cos(a(3)/2)*cos(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*sin(a(1)/2)*sin(a(2)/2);
        q1 = cos(a(3)/2)*sin(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*cos(a(1)/2)*sin(a(2)/2);
        q2 = cos(a(3)/2)*cos(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*sin(a(1)/2)*cos(a(2)/2);
        q3 = cos(a(3)/2)*sin(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*cos(a(1)/2)*cos(a(2)/2);

        q = [q0 ; q1 ; q2 ; q3] / norm([q0 ; q1 ; q2 ; q3]);

        q0 = q(1) ; q1 = q(2) ;q2=q(3) ;q3 = q(4) ;
        cbn = [q0^2+q1^2-q2^2-q3^2,     2*(q1*q2+q0*q3),       2*(q1*q3-q0*q2);
                2*(q1*q2-q0*q3),         q0^2-q1^2+q2^2-q3^2,   2*(q2*q3+q0*q1);
                2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),       q0^2-q1^2-q2^2+q3^2]'; 
end