clear all
clc

syms p q p_dot bg ba px py pz phi theta psi p_dot_x p_dot_y p_dot_z bg_x bg_y bg_z ba_x ba_y ba_z
syms wm_x wm_y wm_z ng_x ng_y ng_z am_x am_y am_z na_x na_y na_z nbg_x nbg_y nbg_z nba_x nba_y nba_z 

p = [px ; py ; pz];
q = [phi ; theta ; psi];
p_dot = [p_dot_x ; p_dot_y ; p_dot_z];
gyro_bias = [bg_x ; bg_y ; bg_z];
acc_bias = [ba_x ; ba_y ; ba_z];

X = [p ; q ; p_dot ; bg ; ba];


G = inv(g(q));
R = r(q);
wm = [wm_x ; wm_y ; wm_z];
ng = [ng_x ; ng_y ; ng_z];
am = [am_x ; am_y ; am_z];
na = [na_x ; na_y ; na_z]; 
nbg = [nbg_x ; nbg_y ; nbg_z]; 
nba = [nba_x ; nba_y ; nba_z]; 
g = [0 ; 0 ; 9.8];

X_dot = [p_dot ; G*( wm - bg - ng) ; g + (R*(am - ba - na)) ; nbg ; nba];

n = [ng ; na ; nbg ; nba];

A_t = jacobian(X_dot,X);
U_t = jacobian(X_dot,n);

%F_t = eye(15) + (A_t*dt);
%V_t = U_t*dt;