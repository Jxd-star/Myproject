function GenerateInitCode2_GeneralStatus(generalstatus)

    clc

    fprintf("\tpgeneralstatus->FirstInit_Flag = true;\n");
    for i = 1 : numel(generalstatus.Mpv)
        if generalstatus.Mpv(i) ~= 0
            fprintf("\tpgeneralstatus->Mpv[%u] = %s; /* %+40.32e */\n", i-1, float64tohexfloat64(generalstatus.Mpv(i)), generalstatus.Mpv(i));
        end
    end
    
end