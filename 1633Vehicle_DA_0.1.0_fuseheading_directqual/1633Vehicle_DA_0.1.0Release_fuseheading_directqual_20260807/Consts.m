classdef Consts
    properties (Constant)
        e = 1 / 298.257;       %地球曲率
        re = 6378137;          %地球半径
        wie = 7.2921151467e-5; %地球自转角速率
        g0 = 9.780325;         %重力
        t = 0.005;             %imu更新时间s
        T = 0.05;              %gps更新时间s
        length_imu_std1_tmp_part5 = int32(1200);
        length_p111_part6 = int32(2400);
    end
end