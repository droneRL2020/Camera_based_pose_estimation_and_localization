function [R, T] = RT_estimation(points_c, points_w, inv_K)
n = size(points_c,2);
M = zeros(2*n, 9);
M(1:n, 1) = -points_w(1, :);
M(1:n, 2) = -points_w(2, :);
M(1:n, 3) = -1;
M(1:n, 7) = points_w(1,:).*points_c(1,:);
M(1:n, 8) = points_w(2,:).*points_c(1,:);
M(1:n, 9) = points_c(1,:);

M(n+1:end, 4) = -points_w(1, :);
M(n+1:end, 5) = -points_w(2, :);
M(n+1:end, 6) = -1;
M(n+1:end, 7) = points_w(1,:).*points_c(2,:);
M(n+1:end, 8) = points_w(2,:).*points_c(2,:);
M(n+1:end, 9) = points_c(2,:);
[U, D, V] = svd(M);
h = V(:,end);
H = reshape(h,3,3)';

if H(3,3) < 0
    H = -H;
end
r12_T = inv_K*H;
h1 = r12_T(:,1);
h2 = r12_T(:,2);
h3 = r12_T(:,3);
UDV = [h1, h2, cross(h1,h2)];
[U,D,V] = svd(UDV);
R = U*[1,0,0;0,1,0;0,0,det(U*V')]*V';
T = h3/norm(h1);