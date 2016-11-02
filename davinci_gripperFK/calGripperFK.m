% Define Constant
W_1 = [0 0 -1]'; W_2 = [0 -1 0]'; W_3 = [-1 0 0]'; W_4 = [0 0 1]';
% syms r theta_1 theta_2 theta_3 theta_4;
% theta = real([theta_1 theta_2 theta_3 theta_4]');
r = sym('r', 'real');
theta = sym('theta', [1 4], 'real');
% theta = [0 0 0 0]';
Q_1 = [0 0 0]'; Q_2 = [0 0 0]'; Q_3 = [0 0 0]'; Q_4 = [0 r 0]';
W = [W_1 W_2 W_3 W_4];
Q = [Q_1 Q_2 Q_3 Q_4];
X_G = [1 0 0]; Y_G = [0 1 0]; Z_G = [0 0 1];
X_N = [0 -1 0]; Y_N = [-1 0 0]; Z_N = [0 0 -1];
R_GN_0 = [dot(X_N, X_G) dot(Y_N, X_G) dot(Z_N, X_G); dot(X_N, Y_G) dot(Y_N, Y_G) dot(Z_N, Y_G);
      dot(X_N, Z_G) dot(Y_N, Z_G) dot(Z_N, Z_G)];
P_GN_0 = [0 r 0]';
G_GN_0 = [R_GN_0 P_GN_0;0 0 0 1];

G_GN = gripperFK(W, Q, G_GN_0, theta)
G_GN = simplify(G_GN)
% pretty(G_GN);