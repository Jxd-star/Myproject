function [imu] = Preprocess_imu(imu1)
    imu = [imu1([4, 3, 5]), deg2rad(imu1(6:8)), imu1(9:end)];
end
