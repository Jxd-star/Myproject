function  [att1 , att_ins1 , vel1 , pos1 , imu1 ] = Static_heading_success(att , vel , pos , imu , bw) 
att1 = att ;
att_ins1  = att ;
% vel1 = [vel(1 : 2) , 0 ];
vel1 = [0 , 0 , 0 ];
pos1 = pos ;
imu1 = imu - bw ;
end