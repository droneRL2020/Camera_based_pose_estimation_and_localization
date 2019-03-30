function matrix_g = g(q)
%this is defined to get the matrix G(q) for Z-X-Y Euler Transformation%

phi = q(1);
theta = q(2);
psi = q(3);


matrix_g = [cos(theta) 0 -(cos(phi)*sin(theta)) ; 0 1 sin(phi) ; sin(theta) 0 (cos(phi)*cos(theta))];

end

