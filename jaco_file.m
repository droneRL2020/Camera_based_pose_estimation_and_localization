clear all
clc

syms px py pz phi theta psi p_dot_x p_dot_y p_dot_z bg_x bg_y bg_z ba_x ba_y ba_z
syms wm_x wm_y wm_z ng_x ng_y ng_z am_x am_y am_z na_x na_y na_z nbg_x nbg_y nbg_z nba_x nba_y nba_z 

p = [px ; py ; pz];
q = [phi ; theta ; psi];
p_dot = [p_dot_x ; p_dot_y ; p_dot_z];
bg = [bg_x ; bg_y ; bg_z];
ba = [ba_x ; ba_y ; ba_z];

X = [p ; q ; p_dot ; bg ; ba];


ng = [ng_x ; ng_y ; ng_z];
na = [na_x ; na_y ; na_z];
nbg = [nbg_x ; nbg_y ; nbg_z];
nba = [nba_x ; nba_y ; nba_z];

N = [ng ; na ; nbg ; nba];

U = [wm_x ; wm_y ; wm_z ; am_x ; am_y ; am_z];
 
X_dot = x_t(X,U,N);

A_t = getA(X_dot,X);
V_t = getV(X_dot,N)

%F_t = eye(15) + (A_t*dt);
%V_t = U_t*dt;