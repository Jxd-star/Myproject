function [gps, goe, firstinit, gps123_prv] = Preprocess_gnss(Heading_Comp , gps1, firstinit, gps123_prv)
    gps = zeros(1, 10);
    gps(1:4) = [gps1(25:26) * pi / 180, gps1(27), gps1(28) * pi / 180]; % Lat(rad)-Lon(rad)-Alt, Heading(rad)
    gps(5:7) = gps1(40:42) * 1000 / 3600;
    gps(8:10) = [gps1(30) * pi / 180, gps1(29) * 1000 / 3600, gps1(28) * pi / 180]; % VE(m/s)-VN(m/s)-VertSpd-TrkGnd(rad)-HorSpd(km/h)-heading(rad)

    gps(4) = gps(4) - Heading_Comp ;
    if gps(4) < 0
        gps(4) = gps(4) + 2 * pi ;
    end
    
    for i = 1:size(gps, 2)
        if isnan(gps(i))
            gps(i) = 0;
        end
    end
    
    if ~firstinit
        if gps(1) == 0
            gps(1:3) = gps123_prv;
        end
    else
        firstinit = false;
    end
    
    gps123_prv = gps(1:3);
    
    goe = gps1([32:33, 35:39,39,34]);%sats qual hdop rms latstd lonstd altstd 
end
