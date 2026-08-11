function [X_filter , P_filter ] = Kalman_Filter(X_filter , P_filter , kalman  , v_odo , Value_odo_flag, kalmanparams, Lever_Arm_ODO , Lever_Arm_NHC)

    if Value_odo_flag == 1   %% 有里程计
    
        if kalman.goe12(2) == 1
            R = diag(kalmanparams.R_Qual1_Diag);
            if kalman.k1 == 1
                R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.002 , 0.01 / 180 * pi  ])^2 ;
            end    
        elseif kalman.goe12(2) == 2  
            R = diag(kalmanparams.R_Qual2_Diag);
        elseif kalman.goe12(2) == 5
            R = diag(kalmanparams.R_Qual5_Diag);
        else
            R = diag(kalmanparams.R_QualE_Diag);
        end
        Q = diag(kalmanparams.Q_Diag);

        % Q = diag([[1 , 1 , 1] * 0.1e-5 , [1 , 1 , 1] * 5e-5]) .^2;

    
    else   %% 无里程计

        if kalman.goe12(2) == 1
            R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.01 , 0.001 / 180 * pi  ])^2 ;
            if kalman.k1 == 1
                R = diag([ [1 , 1 , 1] * 0.0002 , [1 , 1 , 1 ] * 0.001 , 0.001 / 180 * pi  ])^2 ;
                if kalman.goe(8) <= 4 && kalman.goe(8) > 0 && norm(kalman.z(5 : 7)) < 0.15
                    R = diag([ [1 , 1 , 1] * 0.0002 , [1 , 1 , 10000] * 0.01 , 0.001 / 180 * pi  ])^2 ;
                end
            end
        elseif kalman.goe12(2) == 2
            R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.01 , 0.001 / 180 * pi  ])^2 ;
        elseif kalman.goe12(2) == 5
            R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.01 , 0.001 / 180 * pi  ])^2 ;
        else
            R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 10 ] * 0.0005 , 0.001 / 180 * pi  ])^2 ;
            if kalman.goe(8) <= 0.025 || (kalman.flag1 == 0 && kalman.goe(8) <= 0.3)
                R = diag([ [1 , 1 , 1] * 0.0002 , [1 , 1 , 10 ] * 0.00005 , 0.001 / 180 * pi  ])^2 ;
            end
        end
        Q = diag([[1 , 1 , 1] * 0.5e-6 , [1 , 1 , 1] * 5e-5] ) ^ 2 ;

    end
    
    if Value_odo_flag == 0   % 没有里程计信息接入


        if kalman.flag1 == 10
            [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
        else
            if kalman.gnssupdateflag % GPS更新时刻，有GPS更新做量测更新，该时刻没有GPS使用NHC
                if kalman.goe12(2) ~= 0 && kalman.yaw_correct == 1 && kalman.goe(8) ~= 0 ...
                    && ~((kalman.goe12(2) == 4 && (kalman.goe(8) > 1 ||kalman.goe(8) == 0 ))...
                    ||   (kalman.goe12(2) == 5 && (kalman.goe(8) > 1 || kalman.goe(8) == 0 )) ...
                    ||   (kalman.goe12(2) == 2 && (kalman.goe(8) > 4 || kalman.goe(8) == 0)) ...
                    ||   (kalman.goe12(2) == 1 && (kalman.goe(8) > 8  / kalman.k1 || kalman.goe(8) == 0)) ...
                    ||   (kalman.goe12(2) == 0 || kalman.goe(8) == 0) )
                    [X_filter , P_filter] = Kalman_GNSS(kalman , X_filter , P_filter , Q , R ) ;  % GNSS量测更新
                else
                    if kalman.flag1 == 0
                        [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
                    else
                        [X_filter , P_filter] = Kalman_NHC(kalman , X_filter , P_filter , Q , Lever_Arm_NHC) ;  % nhc
                    end
                    
                end
            elseif kalman.gnsstimeoutflag
% % 
                if kalman.flag1 == 0
                    [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
                else
                    [X_filter , P_filter] = Kalman_NHC(kalman , X_filter , P_filter , Q , Lever_Arm_NHC) ;  % GNSS量测更新
                end
            else
                [X_filter , P_filter] = Kalman_time(kalman , X_filter , P_filter , Q) ;   % 时间更新
            end
        end

    
    else   % 有里程计信息接入
        
        if kalman.flag1 == 10
            [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
        else
    
            if         (kalman.odo_start == 1  && kalman.goe12(2) == 4 && (kalman.goe(8) > 0.1 || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0 ))...
                    || (kalman.odo_start == 1  && kalman.goe12(2) == 5 && (kalman.goe(8) > 0.1 || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0 )) ...
                    || (kalman.odo_start == 1  && kalman.goe12(2) == 2 && (kalman.goe(8) > 1   || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0)) ...
                    || (kalman.odo_start == 1  && kalman.goe12(2) == 1 && (kalman.goe(8) > 6 / kalman.k1 || kalman.goe12(1) < 15 || kalman.goe(8) == 0)) ...
                    || (kalman.odo_start == 1  && (kalman.goe12(2) == 0 || kalman.goe(8) == 0)) ...
                    || (kalman.odo_start == 1  &&  kalman.yaw_correct == 0)
                [X_filter , P_filter] = Kalman_odo(kalman , X_filter , P_filter , v_odo , Q , Lever_Arm_ODO) ;

            elseif kalman.gnssupdateflag        % 表示当前有GNSS数据更新，下面做融合

                if        (kalman.goe12(2) == 4 && ~(( kalman.goe(8) > 0.1 || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0 ))   ...
                        || kalman.goe12(2) == 5 && ~(( kalman.goe(8) > 0.1 || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0 ))  ...
                        || kalman.goe12(2) == 2 && ~(( kalman.goe(8) > 1   || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0))  ...
                        || kalman.goe12(2) == 1 && ~(( kalman.goe(8) > 6 / kalman.k1 || kalman.goe12(1) < 15 ||   kalman.goe(8) == 0)) ) ...
                        && kalman.yaw_correct ~= 0 
                    [X_filter , P_filter] = Kalman_GNSS(kalman , X_filter , P_filter , Q , R ) ;
    
                else
                    if kalman.odo_start == 1
    
                        [X_filter , P_filter] = Kalman_odo(kalman , X_filter , P_filter , v_odo , Q , Lever_Arm_ODO) ;
    
                    else
    
                        [X_filter , P_filter] = Kalman_time(kalman , X_filter , P_filter , Q) ;
    
                    end
                end
    
            else   %卡尔曼滤波时间更新，没有量测
                if kalman.odo_start == 1
                    [X_filter , P_filter] = Kalman_odo(kalman , X_filter , P_filter , v_odo , Q , Lever_Arm_ODO) ;
                else
    
                    [X_filter , P_filter] = Kalman_time(kalman , X_filter , P_filter , Q) ;
    
                end
            end
        end
    
    
    end
    
    
    P_filter = (P_filter + P_filter') / 2 ;

end





