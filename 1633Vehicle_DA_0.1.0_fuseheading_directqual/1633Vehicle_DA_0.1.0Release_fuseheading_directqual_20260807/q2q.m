function q = q2q(wnbb , q , t)      %四元数更新及归一化

    dq = [0 , -wnbb(1) , -wnbb(2) ,  -wnbb(3) ;
          wnbb(1) ,  0 ,  wnbb(3) ,  -wnbb(2) ; 
          wnbb(2) ,  -wnbb(3) , 0 ,   wnbb(1) ; 
          wnbb(3) ,   wnbb(2) , -wnbb(1) , 0 ] * q * 0.5 ;

    q = (q + dq * t) / norm(q + dq * t) ;  
end