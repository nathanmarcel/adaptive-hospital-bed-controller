num = [0, 0, 0, 608.2];
denom = [0.01736, 62, 2500, 0.7];
tf_sys = tf(num, denom);
[A, B, C, D] = tf2ss(num, denom);
sys = ss(A, B, C, D);

% LQR Weighting Matrices
% Q = C.' * C;
Q = diag([0 1 0]);
lambda = 1E-8;
R = lambda*diag([1]);

% P = [P1 P2 P3; P4 P5 P6; P7 P8 P9];

% ans = A'*P + P*A + Q - P*B*(1/R)*B'*P;


[K, S, P] = lqr(sys, Q, R);




