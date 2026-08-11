function gps = Lever_arm_GNSS_delay1(gps , cbn , webb , Lever_Arm_GNSS , Mpv , vel , an) 
gps(5 : 7) = gps(5 : 7) - (cbn * (webb * Lever_Arm_GNSS'))' + an * 0.06 ;
gps(1 : 3) = gps(1 : 3) - (Mpv * cbn * Lever_Arm_GNSS')'  + (Mpv * vel')' * 0.06 ;
end