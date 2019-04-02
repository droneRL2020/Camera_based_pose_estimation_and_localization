function A_t = getA(X_dot, X)
%This the coeeficient matrix that is used to calculate F_t
A_t = jacobian(X_dot,X);

end
