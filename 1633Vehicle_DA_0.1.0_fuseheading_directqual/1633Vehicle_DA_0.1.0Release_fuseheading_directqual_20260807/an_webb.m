function [an , webb] = an_webb(v_ins1 , v_ins, wnbb , i , t) 
if i <= 1
    an = [0 , 0 , 0] ;     % 加速度粗略估算
else
    an = (v_ins1 - v_ins) / t ;
end
webb = [0 , -wnbb(3) , wnbb(2) ;     wnbb(3) , 0 , -wnbb(1) ;     -wnbb(2) , wnbb(1) , 0 ] ;
end