function [cbn , q , a] = Initial_alignment2(imu123mean , gps4mean  ) 

aa = zeros(1, 3);
aa(1) = asin(imu123mean(2)) + deg2rad(0) ;
aa(2) = atan(-imu123mean(1) / imu123mean(3)) + deg2rad(0) ;


aa(3) = gps4mean ;   % 初始时航向角---双天线

% aa(3) = 0  / 180 * pi ;  % 初始航向角---单天线手动配置 , 或通过逻辑自寻默认配置为0

a = aa ; 
q0 = cos(a(3)/2)*cos(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*sin(a(1)/2)*sin(a(2)/2);
q1 = cos(a(3)/2)*sin(a(1)/2)*cos(a(2)/2)+sin(a(3)/2)*cos(a(1)/2)*sin(a(2)/2);
q2 = cos(a(3)/2)*cos(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*sin(a(1)/2)*cos(a(2)/2);
q3 = cos(a(3)/2)*sin(a(1)/2)*sin(a(2)/2)-sin(a(3)/2)*cos(a(1)/2)*cos(a(2)/2);

q = [q0 ; q1 ; q2 ; q3] / norm([q0 ; q1 ; q2 ; q3]);

cbn = [q0^2+q1^2-q2^2-q3^2,     2*(q1*q2+q0*q3),       2*(q1*q3-q0*q2);
       2*(q1*q2-q0*q3),         q0^2-q1^2+q2^2-q3^2,   2*(q2*q3+q0*q1);
       2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),       q0^2-q1^2-q2^2+q3^2]'; 


end