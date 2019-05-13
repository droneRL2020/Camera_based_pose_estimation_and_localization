function points = tag_points(index)
%            o ----------> Y(j)
%            | P4 == P3
%            | || P0 ||
%            | P1 == P2
%            v
%            X(i)
%       [j]   0   1  2     3   4   5     6   7   8
% tag_ids =   [0, 12, 24,  36, 48, 60,  72, 84, 96;
%              1, 13, 25,  37, 49, 61,  73, 85, 97;
%              2, 14, 26,  38, 50, 62,  74, 86, 98;
%              3, 15, 27,  39, 51, 63,  75, 87, 99;
%              4, 16, 28,  40, 52, 64,  76, 88, 100;
%              5, 17, 29,  41, 53, 65,  77, 89, 101;
%              6, 18, 30,  42, 54, 66,  78, 90, 102;
%              7, 19, 31,  43, 55, 67,  79, 91, 103;
%              8, 20, 32,  44, 56, 68,  80, 92, 104;
%              9, 21, 33,  45, 57, 69,  81, 93, 105;
%             10, 22, 34,  46, 58, 70,  82, 94, 106;
%             11, 23, 35,  47, 59, 71,  83, 95, 107];
persistent all_p0;
persistent all_p1;
persistent all_p2;
persistent all_p3;
persistent all_p4;

% if isempty(all_p0)
% all_p0 = zeros(2, 108);
% all_p1 = zeros(2, 108);
% all_p2 = zeros(2, 108);
% all_p3 = zeros(2, 108);
% all_p4 = zeros(2, 108);

for id = 0:107
    i = mod(id, 12);
    j = floor(id/12);
    tag_length = 0.152;
    space = 0.152;
    odd_space = 0.178;

    if j >= 3 & j < 6
        p4 = [(tag_length + space) * i;(tag_length+space) * j + (odd_space - space)];
    elseif j >= 6 
        p4 = [(tag_length + space) * i;(tag_length+space) * j + (odd_space - space) * 2];
    else
        p4 = [(tag_length + space) * i;(tag_length+space) * j];
    end

    p0 = p4 + [tag_length/2; tag_length/2];
    p1 = p4 + [tag_length;0];
    p2 = p4 + [tag_length; tag_length];
    p3 = p4 + [0; tag_length];

    all_p0(:,id+1) = p0;
    all_p1(:,id+1) = p1;
    all_p2(:,id+1) = p2;
    all_p3(:,id+1) = p3;
    all_p4(:,id+1) = p4;
end 
% end

index = index+1;
points = [all_p0(:,index), all_p1(:,index), all_p2(:, index), all_p3(:,index), all_p4(:,index)];
