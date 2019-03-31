function x_t = x_t(X,U,N)

p_dot = [X(7) ; X(8); X(9)];
G = inv(g(X(4),X(5),X(6)));
R = r(X(4),X(5),X(6));
wm = [U(1) ; U(2) ; U(3)];
ng = [N(1) ; N(2) ; N(3)];
bg = [X(10) ; X(11) ; X(12)];
am = [U(4) ; U(5) ; U(6)];
na = [N(4) ; N(5) ; N(6)];
ba = [X(13) ; X(14) ; X(15)];
nbg = [N(7) ; N(8) ; N(9)]; 
nba = [N(10) ; N(11) ; N(12)]; 
grav = [0 ; 0 ; 9.8];

x_t = [p_dot ; G*( wm - bg - ng) ; grav + (R*(am - ba - na)) ; nbg ; nba];

end

