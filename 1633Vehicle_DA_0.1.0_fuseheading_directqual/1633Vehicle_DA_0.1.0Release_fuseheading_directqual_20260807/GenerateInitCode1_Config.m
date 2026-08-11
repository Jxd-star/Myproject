function GenerateInitCode1_Config(config)

    clc

    fprintf("\tstatic const Algo_Config_Type config_default = {\n");

    fprintf("\t\t.cvb = {\n");
    for i = 1 : numel(config.cvb)
        if config.cvb(i) ~= 0
            fprintf("\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.cvb(i)), config.cvb(i));
        end
    end
    fprintf("\t\t},\n");

    fprintf("\t\t.cvb_nhc = {\n");
    for i = 1 : numel(config.cvb_nhc)
        if config.cvb_nhc(i) ~= 0
            fprintf("\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.cvb_nhc(i)), config.cvb_nhc(i));
        end
    end
    fprintf("\t\t},\n");

    fprintf("\t\t.Lever_Arm_GNSS = {\n");
    for i = 1 : numel(config.Lever_Arm_GNSS)
        if config.Lever_Arm_GNSS(i) ~= 0
            fprintf("\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.Lever_Arm_GNSS(i)), config.Lever_Arm_GNSS(i));
        end
    end
    fprintf("\t\t},\n");

    fprintf("\t\t.Lever_Arm_ODO = {\n");
    for i = 1 : numel(config.Lever_Arm_ODO)
        if config.Lever_Arm_ODO(i) ~= 0
            fprintf("\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.Lever_Arm_ODO(i)), config.Lever_Arm_ODO(i));
        end
    end
    fprintf("\t\t},\n");

    fprintf("\t\t.Lever_Arm_NHC = {\n");
    for i = 1 : numel(config.Lever_Arm_NHC)
        if config.Lever_Arm_NHC(i) ~= 0
            fprintf("\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.Lever_Arm_NHC(i)), config.Lever_Arm_NHC(i));
        end
    end
    fprintf("\t\t},\n");

    fprintf("\t\t.k0001 = %s, /* %+40.32e */\n", float64tohexfloat64(config.k0001), config.k0001);
    fprintf("\t\t.k0002 = %s, /* %+40.32e */\n", float64tohexfloat64(config.k0002), config.k0002);
    fprintf("\t\t.k0003 = %s, /* %+40.32e */\n", float64tohexfloat64(config.k0003), config.k0003);
    fprintf("\t\t.k0004 = %s, /* %+40.32e */\n", float64tohexfloat64(config.k0004), config.k0004);

    fprintf("\t\t.KalmanParams = {\n");

    fprintf("\t\t\t.P_diag = {\n");
    for i = 1 : numel(config.KalmanParams.P_diag)
        if config.KalmanParams.P_diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.P_diag(i)), config.KalmanParams.P_diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t\t.Q_Diag = {\n");
    for i = 1 : numel(config.KalmanParams.Q_Diag)
        if config.KalmanParams.Q_Diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.Q_Diag(i)), config.KalmanParams.Q_Diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t\t.R_Qual1_Diag = {\n");
    for i = 1 : numel(config.KalmanParams.R_Qual1_Diag)
        if config.KalmanParams.R_Qual1_Diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.R_Qual1_Diag(i)), config.KalmanParams.R_Qual1_Diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t\t.R_Qual2_Diag = {\n");
    for i = 1 : numel(config.KalmanParams.R_Qual2_Diag)
        if config.KalmanParams.R_Qual2_Diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.R_Qual2_Diag(i)), config.KalmanParams.R_Qual2_Diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t\t.R_Qual5_Diag = {\n");
    for i = 1 : numel(config.KalmanParams.R_Qual5_Diag)
        if config.KalmanParams.R_Qual5_Diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.R_Qual5_Diag(i)), config.KalmanParams.R_Qual5_Diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t\t.R_QualE_Diag = {\n");
    for i = 1 : numel(config.KalmanParams.R_QualE_Diag)
        if config.KalmanParams.R_QualE_Diag(i) ~= 0
            fprintf("\t\t\t\t[%u] = %s, /* %+40.32e */\n", i-1, float64tohexfloat64(config.KalmanParams.R_QualE_Diag(i)), config.KalmanParams.R_QualE_Diag(i));
        end
    end
    fprintf("\t\t\t},\n");

    fprintf("\t\t},\n");

    fprintf("\t\t.d_yaw_v2b = %s, /* %+40.32e */\n", float64tohexfloat64(config.d_yaw_v2b), config.d_yaw_v2b);

    fprintf("\t};\n");
    
end