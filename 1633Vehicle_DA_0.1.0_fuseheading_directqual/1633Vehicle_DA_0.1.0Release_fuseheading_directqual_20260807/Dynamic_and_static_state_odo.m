function [flag1 , dong_jing_tai ] = Dynamic_and_static_state_odo(flag1 , imu) 
if abs(mean([imu(11) , imu(10)])) > 0.02
    flag1 = int32(1);      %动态标志位
elseif abs(mean([imu(11) , imu(10)])) <= 0.02
    flag1 = int32(0);      %静态标志位
end
dong_jing_tai = flag1 ;
end