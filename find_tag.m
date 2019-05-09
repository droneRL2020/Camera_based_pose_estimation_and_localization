function [x,y] = find_tag(number)
    x = mod(number,12) * 0.152 * 2;
    y_cord = floor(number/12);
    y = y_cord * 0.152 * 2;
    if y_cord > 2
        y = y + 0.26;
    end
    if y_cord > 5
        y = y + 0.26;
    end
end