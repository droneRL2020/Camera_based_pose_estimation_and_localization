function V_t = getV(X_dot, N)

V_t = jacobian(X_dot,N);

end
