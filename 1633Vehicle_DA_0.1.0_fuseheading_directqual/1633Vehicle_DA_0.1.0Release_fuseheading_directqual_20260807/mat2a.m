function a = mat2a(cbn) 


if abs(cbn(3,2)) < 0.999999
    a = [ asin(cbn(3,2)); -atan2(cbn(3,1),cbn(3,3)); atan2(cbn(1,2),cbn(2,2)) ]';
else
    a = [ asin(cbn(3,2));  atan2(cbn(1,3),cbn(1,1));  0 ]';
end

if a(3) < 0 
    a(3) = a(3) + 2 * pi ;
end


end

