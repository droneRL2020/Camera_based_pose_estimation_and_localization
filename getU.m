function U_t = getU(X_dot, N)
% This is U_t matrix is not the control matrix to be passed to the function
% this the matrix which comes before the noise vector
U_t = jacobian(X_dot,N);

end
