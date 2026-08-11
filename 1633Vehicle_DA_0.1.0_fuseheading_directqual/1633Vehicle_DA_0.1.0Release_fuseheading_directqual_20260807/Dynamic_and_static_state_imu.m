
function [flag1 , dong_jing_tai , k0 ] = Dynamic_and_static_state_imu(flag1 , imu , k0 , bw , xita , p1 , p1111)  %动静态标志位 ，imu-------里程计判断的
if  flag1 == 0
    if abs(norm(imu(1 : 3))   - 1) > xita(1) || p1(1) > p1111(1) * 1  %% 这个判断是GPS存在 ， 如果GPS不存在怎么处理
        k0 = k0 + int32(1) ;
    else
        k0 = int32(0) ;
    end
end
if  flag1 == 1
    if (norm(imu(4 : 6)  - bw))  <= xita(2) * 1.0 && abs(norm(imu(1 : 3))   - 1) < xita(1) * 1.0  && p1(1) < p1111(1) * 1    %% 这个判断是GPS存在 ， 如果GPS不存在怎么处理
        k0 = k0 + int32(1) ;
    else
        k0 = int32(0) ;
    end
end
if flag1 == 0 && k0 == 20     % 使用NHC时，动态判断一定要准，及时
    flag1 = int32(1) ;
    k0 = int32(0) ;
elseif flag1 == 1 && k0 == 20
    flag1 = int32(0) ;
    k0 = int32(0) ;
end

dong_jing_tai = flag1 ;


end