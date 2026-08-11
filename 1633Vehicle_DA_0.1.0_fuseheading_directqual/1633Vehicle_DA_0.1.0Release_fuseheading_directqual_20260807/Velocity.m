function [v0 , fnn] = Velocity(v , fn ,wie , p , t , g , rm , rn) 

%     v0(1) = v(1) + (fn(1) + v(2) * (2 * wie * sin(p(1)) + v(1) / (rn + p(3)) * tan(p(1))) - ...      %速度解算
%                v(3) * (2 * wie * cos(p(1)) + v(1) / (rn + p(3)))) * t ;
% 
%     v0(2) = v(2) + (fn(2) - v(1) * (2 * wie * sin(p(1)) + v(1) / (rn + p(3)) * tan(p(1))) - ...
%                v(2) / (rm + p(3)) * v(3)) * t ;
% 
%     v0(3) = v(3) + (fn(3) + v(1) * (2 * wie * cos(p(1)) + v(1) / (rn + p(3))) + v(2) ^ 2 / (rm + p(3)) - g) * t ;


wie1 = [ 0 , wie * cos(p(1)) , wie * sin(p(1))] ;
wet = [ - v(2) / (rm + p(3)) , v(1) / (rn + p(3)) , v(1) / (rn + p(3))  * tan(p(1))] ;

fnn = fn' - cross(2 * wie1 + wet , v) - [0 , 0 , g] ;

v0 = v + fnn * t ;


end