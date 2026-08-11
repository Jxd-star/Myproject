function [array] = Append_Ascending(array, data, num)

low = zeros(1, 1, class(num));
high = zeros(1, 1, class(num));
low(1, 1) = 1;
high(1, 1) = num;

while low <= high
    mid = bitshift(low + high, -1);
    if array(mid) > data
        high = mid - 1;
    else
        low = mid + 1;
    end
end
for i = num : -1 : low
    array(i + 1) = array(i);
end
array(low) = data;

end
