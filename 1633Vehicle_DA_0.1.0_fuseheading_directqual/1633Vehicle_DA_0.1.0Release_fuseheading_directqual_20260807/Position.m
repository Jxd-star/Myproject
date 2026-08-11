function p0 = Position(rm , rn , p , v , v1 , t) 
    v = (v + v1) / 2 ;
    p0 = zeros(1, 3);
    p0(1) = p(1) + v(2) / (rm + p(3)) * t ;   %位置解算
    p0(2) = p(2) + v(1) / (rn + p(3)) * sec(p(1)) * t ;
    p0(3) = p(3) + v(3) * t ;
end