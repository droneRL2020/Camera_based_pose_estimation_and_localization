function z_c = z_c_estimation(wRc, wTc, points, inv_k)
u = points(:,1);
v = points(:,2);
n = numel(u);
xyz_c= inv_k*[u';v';ones(1,n)];
tmp1 = wRc'*wTc;
tmp1 = tmp1(3);
tmp2 = wRc'*xyz_c;
tmp2 = tmp2(3,:);
lambda = tmp1./tmp2;
z_c = ones(1,n).*lambda;
end