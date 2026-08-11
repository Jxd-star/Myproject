function gps = Lever_arm_GNSS_delay(gps , Mpv , webb , vel , Yaw_rate_mean , yaw_rate , i , cbn , Lever_Arm_GNSS , an)



gps(1 : 3) = gps(1 : 3) - (Mpv * cbn * Lever_Arm_GNSS)'  + (Mpv * vel')' * 0.06 ;
gps(5 : 7) = gps(5 : 7) - (cbn * (webb * Lever_Arm_GNSS))' + an * 0.06  ;

if i > 30
    gps(8) = gps(8) + Yaw_rate_mean * 0.11  ;  % 航迹补偿
    gps(4) = gps(4) + Yaw_rate_mean * 0.06  ;  % 航向补偿
else
    gps(8) = gps(8) + yaw_rate * 0.11  ;  % 航迹补偿
    gps(4) = gps(4) + Yaw_rate_mean * 0.06  ;  % 航向补偿
end

if gps(8) > 2 * pi
    gps(8) = gps(8) - 2 * pi ;
elseif gps(8) < 0
    gps(8) = gps(8) + 2 * pi ;
end

if gps(4) > 2 * pi
    gps(4) = gps(4) - 2 * pi ;
elseif gps(4) < 0
    gps(4) = gps(4) + 2 * pi ;
end



end