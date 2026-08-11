function GenerateInitCode2_MainStepStatus(mainstepstatus)

    clc

    fprintf("\tpmainstepstatus->imu6_1_idx = %dL;\n", mainstepstatus.imu6_1_idx);
    fprintf("\tpmainstepstatus->imu6_2_idx = %dL;\n", mainstepstatus.imu6_2_idx);
    fprintf("\tpmainstepstatus->gps8_1_idx = %dL;\n", mainstepstatus.gps8_1_idx);
    fprintf("\tpmainstepstatus->gps8_2_idx = %dL;\n", mainstepstatus.gps8_2_idx);
    fprintf("\tpmainstepstatus->imu456_sum_idx = %dL;\n", mainstepstatus.imu456_sum_idx);
    fprintf("\tpmainstepstatus->imu456_sum_cnt_recalc = %dL;\n", mainstepstatus.imu456_sum_cnt_recalc);
    fprintf("\tpmainstepstatus->RecalculateGyroBiasStatus.imuinput456_idx = %dL;\n", mainstepstatus.RecalculateGyroBiasStatus.imuinput456_idx);
    fprintf("\tpmainstepstatus->OdoExtParamsCalibStatus.x_filter202122_idx = %dL;\n", mainstepstatus.OdoExtParamsCalibStatus.x_filter202122_idx);

    fprintf("\tpmainstepstatus->k1 = %dL;\n", mainstepstatus.k1);
    for i = 1 : size(mainstepstatus.p_filter, 1)
        fprintf("\tpmainstepstatus->p_filter[%d] = pctrltab->pConfig->KalmanParams.P_diag[%d];\n", (i-1)*(size(mainstepstatus.p_filter, 1) + 1), i-1);
    end
    fprintf("\tpmainstepstatus->HeadingFusionActiveFlag = true;\n");

    fprintf("\tpmainstepstatus->imu456sbwabsmax_idx = %dL;\n", mainstepstatus.imu456sbwabsmax_idx);
    fprintf("\tpmainstepstatus->gps9_idx = %dL;\n", mainstepstatus.gps9_idx);
    fprintf("\tpmainstepstatus->imu456_idx = %dL;\n", mainstepstatus.imu456_idx);
    fprintf("\tpmainstepstatus->DetermineTrackStatus.yaw_correct1_prv = %dL;\n", mainstepstatus.DetermineTrackStatus.yaw_correct1_prv);
    fprintf("\tpmainstepstatus->k_xita = %dL;\n", mainstepstatus.k_xita);
end