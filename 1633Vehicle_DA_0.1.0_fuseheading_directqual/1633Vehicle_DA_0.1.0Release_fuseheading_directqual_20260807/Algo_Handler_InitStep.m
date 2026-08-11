function [generalstatus, initstepstatus, output] = Algo_Handler_InitStep(config, generalstatus, initstepstatus, tempsensorvaluetab, output)
    if initstepstatus.counterimu < intmax("int32")
        initstepstatus.counterimu = initstepstatus.counterimu+int32(1);
    end
    if tempsensorvaluetab.gnssupdateflag
        if initstepstatus.countergnss < intmax("int32")
            initstepstatus.countergnss = initstepstatus.countergnss+int32(1);
        end
    end

    if ~initstepstatus.initflags_part1
        if (initstepstatus.counterimu >= int32(400)) && (initstepstatus.counterimu <= int32(2400))
            initstepstatus.imu123sum_part1 = initstepstatus.imu123sum_part1 + tempsensorvaluetab.imu(1:3);
        end
        if initstepstatus.counterimu >= int32(2400)
            [generalstatus.cbn, generalstatus.q, generalstatus.att] = Initial_alignment2(initstepstatus.imu123sum_part1 / 2001, 0);
            initstepstatus.initflags_part1 = true;

            initstepstatus.skipoutputcalcflag = true;

            output.att = generalstatus.att;
            output.updateflags_att = true;
            output.att_ins = generalstatus.att;
            output.updateflags_att_ins = true;
        end
    end
   
    if ~initstepstatus.initflags_part4
        if (initstepstatus.counterimu >= int32(400)) && (initstepstatus.counterimu <= int32(2400))
            initstepstatus.imu456sum_part4 = initstepstatus.imu456sum_part4 + tempsensorvaluetab.imu(4:6);
        end
        if (initstepstatus.counterimu >= int32(2400))
            generalstatus.bw = initstepstatus.imu456sum_part4 / 2001;
            initstepstatus.initflags_part4 = true;
        end
    end
    
    if ~initstepstatus.initflags_part5
        if (initstepstatus.counterimu >= int32(2401)) && (initstepstatus.counterimu <= int32(2401 + Consts.length_imu_std1_tmp_part5 - 1))
            imu_123_norm_s1_abs = abs(norm(tempsensorvaluetab.imu(1:3))-1);
            imu_456_sbw_norm = norm(tempsensorvaluetab.imu(4:6)-generalstatus.bw);
            initstepstatus.imu_std1_part5(:, 1) = Append_Ascending(initstepstatus.imu_std1_part5(:, 1), imu_123_norm_s1_abs, initstepstatus.counterimu - 2400 - 1);
            initstepstatus.imu_std1_part5(:, 2) = Append_Ascending(initstepstatus.imu_std1_part5(:, 2), imu_456_sbw_norm, initstepstatus.counterimu - 2400 - 1);
        end
        if (initstepstatus.counterimu >= int32(2401 + Consts.length_imu_std1_tmp_part5 - 1))
            generalstatus.xita = initstepstatus.imu_std1_part5(round(Consts.length_imu_std1_tmp_part5) * 0.92, :);
            initstepstatus.initflags_part5 = true;
        end
    end
    
    if ~initstepstatus.initflags_part6
        if (initstepstatus.counterimu >= int32(19-10+1)) && (initstepstatus.counterimu <= int32(1 + Consts.length_p111_part6 - 1))
            initstepstatus.y1_part6 = [initstepstatus.y1_part6(2:end, :); sqrt(tempsensorvaluetab.imu(4 : 6) * tempsensorvaluetab.imu(4 : 6)')];
            if initstepstatus.counterimu > 19
                x1 = (1: 1: 11)';
                p11 = abs(polyfit(x1, initstepstatus.y1_part6, 2));
            else
                p11 = zeros(1, 3);
            end
            p111_abs = abs(p11);
            initstepstatus.p111_abs_part6(:, 1) = Append_Ascending(initstepstatus.p111_abs_part6(:, 1), p111_abs(1), initstepstatus.counterimu - 1);
            initstepstatus.p111_abs_part6(:, 2) = Append_Ascending(initstepstatus.p111_abs_part6(:, 2), p111_abs(2), initstepstatus.counterimu - 1);
            initstepstatus.p111_abs_part6(:, 3) = Append_Ascending(initstepstatus.p111_abs_part6(:, 3), p111_abs(3), initstepstatus.counterimu - 1);
        end
        if (initstepstatus.counterimu >= int32(1 + Consts.length_p111_part6 - 1))
            generalstatus.p1111 = initstepstatus.p111_abs_part6(round(Consts.length_p111_part6 * 0.995), :);
            initstepstatus.initflags_part6 = true;
        end
    end

    if ~initstepstatus.skipoutputcalcflag
        if initstepstatus.initflags_part4 && initstepstatus.initflags_part1
            wnbb = (tempsensorvaluetab.imu(4:6) - generalstatus.bw)';    %陀螺仪
            if generalstatus.Value_odo_flag == int32(1) %有里程计

                [generalstatus.flag1, output.dong_jing_tai] = Dynamic_and_static_state_odo(generalstatus.flag1, tempsensorvaluetab.imu); %动静态标志位 ，里程计判断的
                output.updateflags_dong_jing_tai = true;

                if generalstatus.flag1 == 0 && generalstatus.gps_latest(9) > 0.1 && generalstatus.goe_latest(3) <= 1 && (norm(tempsensorvaluetab.imu(1 : 3)) > 1.02)
                    generalstatus.flag1 = int32(1);
                    output.dong_jing_tai = int32(1);
                    output.updateflags_dong_jing_tai = true;
                end
                if generalstatus.flag1 == int32(0)
                    wnbb(:) = 0;
                end
                output.updateflags_dong_jing_tai = true;   % 没有 else 不输出实际跑车运行有没有影响？
            end
            generalstatus.q = q2q(wnbb, generalstatus.q, Consts.t);
            %% cbn更新
            generalstatus.cbn = q2mat(generalstatus.q);
            generalstatus.att = mat2a(generalstatus.cbn);  % 姿态角更新
            output.att = generalstatus.att;
            output.updateflags_att = true;
        end
    else
        initstepstatus.skipoutputcalcflag = false;
    end

     if ~initstepstatus.initflags_part3
        if (tempsensorvaluetab.gnssupdateflag) && (initstepstatus.countergnss >= int32(260)) && tempsensorvaluetab.goe(8) > 0 && tempsensorvaluetab.gps(4) > 1e-6 && tempsensorvaluetab.goe(9) == 3 ... 
                && (tempsensorvaluetab.goe12(2) == 4 && tempsensorvaluetab.goe(8) < 1 || tempsensorvaluetab.goe12(2) ~= 4 && tempsensorvaluetab.goe12(2) > 0 && tempsensorvaluetab.goe(8) < 5)
            initstepstatus.countergnss_value = initstepstatus.countergnss_value + int32(1);
            initstepstatus.gps12345sum_part3(1:3) = initstepstatus.gps12345sum_part3(1:3) + tempsensorvaluetab.gps(1:3);
            initstepstatus.gps12345sum_part3(4) = initstepstatus.gps12345sum_part3(4) + cos(tempsensorvaluetab.gps(4));
            initstepstatus.gps12345sum_part3(5) = initstepstatus.gps12345sum_part3(5) + sin(tempsensorvaluetab.gps(4));
        end
        if (tempsensorvaluetab.gnssupdateflag) && (initstepstatus.countergnss_value >= int32(100)) && (initstepstatus.initflags_part1)
            if norm(tempsensorvaluetab.gps(5:7)) > 1 && tempsensorvaluetab.gps(4) > 1e-6 && tempsensorvaluetab.goe(9) == 3
                generalstatus.pos = tempsensorvaluetab.gps(1:3);
                generalstatus.att(3) = tempsensorvaluetab.gps(4); % 双天线初始航向角
            else
                generalstatus.pos = (initstepstatus.gps12345sum_part3(1 : 3) / 100);
                cos_ave = initstepstatus.gps12345sum_part3(4)/100;
                sin_ave = initstepstatus.gps12345sum_part3(5)/100;

                generalstatus.att(3) = atan2(sin_ave,cos_ave); % 双天线初始航向角
                if generalstatus.att(3)<0
                    generalstatus.att(3) = generalstatus.att(3) + 2 * pi;
                end

                % if abs(generalstatus.att(3) - tempsensorvaluetab.gps(4)) > deg2rad(5) && tempsensorvaluetab.gps(4) > 1e-6
                %     generalstatus.att(3) = tempsensorvaluetab.gps(4); % 双天线初始航向角
                % end
            end

            output.att(3) = generalstatus.att(3) ;   % 初始时航向角---双天线
            q0 = cos(output.att(3)/2)*cos(output.att(1)/2)*cos(output.att(2)/2)+sin(output.att(3)/2)*sin(output.att(1)/2)*sin(output.att(2)/2);
            q1 = cos(output.att(3)/2)*sin(output.att(1)/2)*cos(output.att(2)/2)+sin(output.att(3)/2)*cos(output.att(1)/2)*sin(output.att(2)/2);
            q2 = cos(output.att(3)/2)*cos(output.att(1)/2)*sin(output.att(2)/2)-sin(output.att(3)/2)*sin(output.att(1)/2)*cos(output.att(2)/2);
            q3 = cos(output.att(3)/2)*sin(output.att(1)/2)*sin(output.att(2)/2)-sin(output.att(3)/2)*cos(output.att(1)/2)*cos(output.att(2)/2);

            generalstatus.q = [q0 ; q1 ; q2 ; q3] / norm([q0 ; q1 ; q2 ; q3]);

            generalstatus.cbn = [q0^2+q1^2-q2^2-q3^2,     2*(q1*q2+q0*q3),       2*(q1*q3-q0*q2);
                2*(q1*q2-q0*q3),         q0^2-q1^2+q2^2-q3^2,   2*(q2*q3+q0*q1);
                2*(q1*q3+q0*q2),         2*(q2*q3-q0*q1),       q0^2-q1^2-q2^2+q3^2]';

            output.att = generalstatus.att;
            output.updateflags_att = true;
            output.att_ins = generalstatus.att;
            output.updateflags_att_ins = true;

            initstepstatus.initflags_part3 = true;

            output.vel = zeros(1 , 3) ;
            if norm(tempsensorvaluetab.gps(5 : 7)) > 0.5
                output.vel = tempsensorvaluetab.gps(5 : 7) ;
            end
            Mpv = [0, 1 / Consts.re, 0;     1 / (Consts.re * cos(generalstatus.pos(1))), 0, 0;     0, 0, 1];
            output.pos = generalstatus.pos - (Mpv * generalstatus.cbn * config.Lever_Arm_GNSS)'  + (Mpv * output.vel')' * 0.06 ;
            generalstatus.pos = output.pos ;

            %%
            output.updateflags_vel = true;
            output.updateflags_pos = true;
        end
    end



end