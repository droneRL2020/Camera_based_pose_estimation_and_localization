function matrix_r = r(q)
   
%to get the rotation matrix for Z-X-Y euler transformation

phi = q(1);
theta = q(2);
psi = q(3);



comp_1 = (cos(psi)*cos(theta)) - (sin(phi)*sin(psi)*sin(theta));
comp_2 = -(cos(phi)*cos(psi));
comp_3 = (cos(psi)*sin(theta)) + (cos(theta)*sin(phi)*sin(psi));
comp_4 = (cos(theta)*sin(psi)) + (cos(psi)*sin(phi)*sin(theta));
comp_5 = cos(phi)*cos(psi);
comp_6 = (sin(psi)*sin(theta)) - (cos(psi)*cos(theta)*sin(phi));
comp_7 = -(cos(phi)*sin(theta));
comp_8 = sin(phi);
comp_9 = cos(phi)*cos(theta);

matrix_r = [comp_1 comp_2 comp_3 ; comp_4 comp_5 comp_6 ; comp_7 comp_8 comp_9];


end
