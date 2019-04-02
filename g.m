function matrix_g = g(phi, theta, psi)
%this is defined to get the matrix G(q) for Z-X-Y Euler Transformation%
% symboli r
matrix_g = [cos(theta) 0 -(cos(phi)*sin(theta)) ; 0 1 sin(phi) ; sin(theta) 0 (cos(phi)*cos(theta))];

end

