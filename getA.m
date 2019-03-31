function A_t = getA(X_dot, X)

A_t = jacobian(X_dot,X);

end
