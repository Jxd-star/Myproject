function [Yaw_rate, yaw_rate , an] = Yaw_rate_and_an(v_ins1 , v_ins , t , att_ins1 , att_ins , i)
if i <= 1
    an = [0 , 0 , 0]' ;     % 加速度粗略估算
else
    an = (v_ins1 - v_ins) / t ;
end