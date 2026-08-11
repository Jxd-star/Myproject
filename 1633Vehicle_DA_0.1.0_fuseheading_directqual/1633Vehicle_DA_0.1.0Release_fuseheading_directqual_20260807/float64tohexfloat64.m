function hexfloat64 = float64tohexfloat64(float64)

    if float64 == 0
        hexfloat64 = "+0x0.0000000000000p+0000";
    else
        tmp = typecast(float64, "uint64");

        hexfloat64 = "";
        if bitand(tmp, 0x8000000000000000) ~= 0
            hexfloat64 = hexfloat64 + "-";
        else
            hexfloat64 = hexfloat64 + "+";
        end
        hexfloat64 = hexfloat64 + sprintf("0x1.%013xp%+05d", bitand(tmp, 0xFFFFFFFFFFFFF), int64(bitand(bitshift(tmp, -52), 0x00000000000007FF))-int64(0x00000000000003FF));
    end

end