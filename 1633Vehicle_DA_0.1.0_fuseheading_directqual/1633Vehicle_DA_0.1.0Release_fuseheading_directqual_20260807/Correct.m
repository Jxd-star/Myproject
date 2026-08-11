function [a , v , p , cbn , q] = Correct(a , v , p , x_filter , q)


    q = qdelphi(q , x_filter(1 : 3));   % 四元数---动态时反馈 ;
    v = v - x_filter(4 : 6)' ;
    p = p - x_filter(7 : 9)' ;

    qnb = q / norm(q);
    q11 = qnb(1)*qnb(1); q12 = qnb(1)*qnb(2); q13 = qnb(1)*qnb(3); q14 = qnb(1)*qnb(4);
    q22 = qnb(2)*qnb(2); q23 = qnb(2)*qnb(3); q24 = qnb(2)*qnb(4);
    q33 = qnb(3)*qnb(3); q34 = qnb(3)*qnb(4);
    q44 = qnb(4)*qnb(4);
    cbn = [ q11+q22-q33-q44,  2*(q23-q14),     2*(q24+q13);
        2*(q23+q14),      q11-q22+q33-q44, 2*(q34-q12);
        2*(q24-q13),      2*(q34+q12),     q11-q22-q33+q44 ];
end


