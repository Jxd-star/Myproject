function [determinetrack_status, yaw_correct1] = DetermineTrack(determinetrack_status , tempsensorvaluetab)
    
    if tempsensorvaluetab.gnssupdateflag   %zhoucheng
        yaw_correct1 = int32(1);
        determinetrack_status.yaw_correct1_prv = yaw_correct1;
    else
        yaw_correct1 = determinetrack_status.yaw_correct1_prv;
    end
    
    if tempsensorvaluetab.gnsstimeoutflag   %zhoucheng
        yaw_correct1 = int32(0);
        determinetrack_status.yaw_correct1_prv = yaw_correct1;
    end
    
    
    if tempsensorvaluetab.gnssupdateflag
        gps38 = [tempsensorvaluetab.gps(3), tempsensorvaluetab.gps(8)];
        
        if determinetrack_status.gps38_prv_validflag
            gps38eqprv = isequal(determinetrack_status.gps38_prv, gps38);
            if gps38eqprv %两拍航迹一样（不准的标志）---zhoucehng
                yaw_correct1 = int32(0);
            end
        end
        
        determinetrack_status.gps38_prv = gps38;
        determinetrack_status.gps38_prv_validflag = true;
    end
    
end