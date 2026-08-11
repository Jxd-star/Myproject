function [config, odoextparamscalib_status, x_filter, p_filter, configupdateflag] = OdoExtParamsCalib(config, odoextparamscalib_status, tempsensorvaluetab, vel, x_filter, p_filter, configupdateflag)
    
    if tempsensorvaluetab.gnssupdateflag
        if norm(vel(1 : 2)) > 5 && tempsensorvaluetab.goe(2) >= 4
            if odoextparamscalib_status.i_Xfilter_odo_angle_k < intmax("int32")
                odoextparamscalib_status.i_Xfilter_odo_angle_k = odoextparamscalib_status.i_Xfilter_odo_angle_k + int32(1);
            end
            
            element = permute(x_filter(16 : 18), [3, 2, 1]);
            elemnum = int32(size(odoextparamscalib_status.x_filter202122_rcd, 1) * size(odoextparamscalib_status.x_filter202122_rcd, 2));
            groupelemidx = int32(mod(odoextparamscalib_status.x_filter202122_idx - int32(1), size(odoextparamscalib_status.x_filter202122_rcd, 1))) + int32(1);
            groupidx = int32((odoextparamscalib_status.x_filter202122_idx - groupelemidx) / size(odoextparamscalib_status.x_filter202122_rcd, 1)) + int32(1);
            if odoextparamscalib_status.x_filter202122_idx < elemnum
                odoextparamscalib_status.x_filter202122_sum = odoextparamscalib_status.x_filter202122_sum - odoextparamscalib_status.x_filter202122_rcd(groupelemidx, groupidx, : );
                odoextparamscalib_status.x_filter202122_sum = odoextparamscalib_status.x_filter202122_sum + element;
                odoextparamscalib_status.x_filter202122_sum_recalc = odoextparamscalib_status.x_filter202122_sum_recalc + element;
            else
                odoextparamscalib_status.x_filter202122_sum = odoextparamscalib_status.x_filter202122_sum_recalc + element;
                odoextparamscalib_status.x_filter202122_sum_recalc(:, :, :) = 0;
            end
            odoextparamscalib_status.x_filter202122_rcd(groupelemidx, groupidx, : ) = element;
            odoextparamscalib_status.x_filter202122_idx = mod(odoextparamscalib_status.x_filter202122_idx, elemnum) + int32(1);
            odoextparamscalib_status.x_filter202122_groupmax(1, groupidx, : ) = max(odoextparamscalib_status.x_filter202122_rcd( : , groupidx, : ), [], 1);
            odoextparamscalib_status.x_filter202122_groupmin(1, groupidx, : ) = min(odoextparamscalib_status.x_filter202122_rcd( : , groupidx, : ), [], 1);
        end
    end
    
    if odoextparamscalib_status.i_Xfilter_odo_angle_k >= 1800
        max_Xfilter_odo_angle_k_kkk = max(odoextparamscalib_status.x_filter202122_groupmax(:, :, 2)) ;
        min_Xfilter_odo_angle_k_kkk = min(odoextparamscalib_status.x_filter202122_groupmin(:, :, 2)) ;
        
        max_Xfilter_odo_angle_k_angle1 = max(odoextparamscalib_status.x_filter202122_groupmax(:, :, 1)) ;
        min_Xfilter_odo_angle_k_angle1 = min(odoextparamscalib_status.x_filter202122_groupmin(:, :, 1)) ;
        
        max_Xfilter_odo_angle_k_angle2 = max(odoextparamscalib_status.x_filter202122_groupmax(:, :, 3)) ;
        min_Xfilter_odo_angle_k_angle2 = min(odoextparamscalib_status.x_filter202122_groupmin(:, :, 3)) ;
        if   max_Xfilter_odo_angle_k_kkk    - min_Xfilter_odo_angle_k_kkk    < 5e-4 ...
                && max_Xfilter_odo_angle_k_angle1 - min_Xfilter_odo_angle_k_angle1 < 1e-3 ...
                && max_Xfilter_odo_angle_k_angle2 - min_Xfilter_odo_angle_k_angle2 < 1e-3 ...
            
            elemnum = size(odoextparamscalib_status.x_filter202122_rcd, 1) * size(odoextparamscalib_status.x_filter202122_rcd, 2);
            odo_k = odoextparamscalib_status.x_filter202122_sum(2) / elemnum;
            odo_angle1 = odoextparamscalib_status.x_filter202122_sum(1) / elemnum;
            odo_angle2 = odoextparamscalib_status.x_filter202122_sum(3) / elemnum;
            
            %
            config.cvb = [1.00000000 , -odo_angle2 ,  0.00000000 ;
                odo_angle2 ,  1.00000000 , -odo_angle1 ;
                0.00000000 ,  odo_angle1 ,  1.00000000] ;
            
            config.d_yaw_v2b = odo_angle2 ;
            d_odo_k   = odo_k ;
            
            config.k0001 = config.k0001 * (1 - d_odo_k) ;
            config.k0002 = config.k0002 * (1 - d_odo_k) ;
            config.k0003 = config.k0003 * (1 - d_odo_k) ;
            config.k0004 = config.k0004 * (1 - d_odo_k) ;
            
            x_filter(16 : 18) = 0 ;
            
            p_filter(16 : 18 , 16 : 18) = diag([1 , 1 , 1] * 0.0002) ^ 2 ;
            odoextparamscalib_status.ExtParamsCalib_RunningFlag = false;
            configupdateflag = true;
            
        end
    end
    
end