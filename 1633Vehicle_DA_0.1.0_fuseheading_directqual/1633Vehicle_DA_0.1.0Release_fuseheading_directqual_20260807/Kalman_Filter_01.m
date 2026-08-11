function [X_filter , P_filter ] = Kalman_Filter_01(X_filter , P_filter , kalman)

kalman.z(4) = kalman.z(8) ;

if kalman.goe(2) == 1

    R = diag([ [1 , 1 , 1] * 0.00005 , [1 , 1 , 1 ] * 0.00005 , 0.0005 / 180 * pi  ])^2 ;

elseif kalman.goe(2) == 2

    R = diag([ [1 , 1 , 1] * 0.00005 , [1 , 1 , 1 ] * 0.00001 , 0.0005 / 180 * pi  ])^2 ;

elseif kalman.goe(2) == 5

    R = diag([ [1 , 1 , 1] * 0.00005 , [1 , 1 , 1 ] * 0.000008 , 0.0005 / 180 * pi  ])^2 ;

else

    R = diag([ [1 , 1 , 1] * 0.00005 , [1 , 1 , 1 ] * 0.000002 , 0.0005 / 180 * pi  ])^2 ;

end

Q = diag([[3 , 5 , 0.5] * 0.5e-5 , [1 , 1 , 1] * 8e-4] * 1) ^ 2 ;

%% 静态参数
% if kalman.flag1 == 0
% 
%     Q = diag([[10 , 10 , 1] * 0.5e-5 , [1 , 1 , 0.1] * 8e-4] * 1) ^ 2 ;
% 
%     if kalman.goe(2) == 1
% 
%         R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.05 , 0.05 / 180 * pi  ])^2 ;
% 
%     elseif kalman.goe(2) == 2
% 
%         R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.02 , 0.05 / 180 * pi  ])^2 ;
% 
%     elseif kalman.goe(2) == 5
% 
%         R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.008 , 0.05 / 180 * pi  ])^2 ;
% 
%     else
% 
%         R = diag([ [1 , 1 , 1] * 0.02 , [1 , 1 , 1 ] * 0.005 , 0.05 / 180 * pi  ])^2 ;
% 
%     end
% end



    if mod(kalman.i - 1 , Consts.T / Consts.t) == 0
        if kalman.goe(2) ~= 0
            if (kalman.goe(1) < 10) ...
                    || (kalman.goe(2) == 4 && (kalman.goe(8) > 0.15 || kalman.goe(8) == 0 )) ...
                    || (kalman.goe(2) == 2 && (kalman.goe(8) > 3 || kalman.goe(8) == 0 )) ...
                    || (kalman.goe(2) == 1 && (kalman.goe(8) > 8 || kalman.goe(8) == 0 )) ...
                    || (kalman.goe(2) == 5 && (kalman.goe(8) > 0.5|| kalman.goe(8) == 0 ))


                if kalman.flag1 == 0
                    [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
                else
                    [X_filter , P_filter] = Kalman_NHC(kalman , X_filter , P_filter , Q) ;  % GNSS量测更新
                end

            else
                if kalman.flag1 == 0
                    [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
                else
                    [X_filter , P_filter] = Kalman_GNSS(kalman , X_filter , P_filter , Q , R ) ;  % GNSS量测更新
                end
            end
        else
            if kalman.flag1 == 0
                [X_filter , P_filter] = Kalman_ZUPT(kalman , X_filter , P_filter , Q) ;
            else
                [X_filter , P_filter] = Kalman_NHC(kalman , X_filter , P_filter , Q) ;  % GNSS量测更新
            end
        end

    else

        [X_filter , P_filter] = Kalman_time(kalman , X_filter , P_filter , Q) ;   % 时间更新

    end




P_filter = (P_filter + P_filter') / 2 ;
end








