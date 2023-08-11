num = [0, 0, 0, 608.2];
denom = [0.01736, 62, 2500, 0.7];
p = tf(num, denom);

[A, B, C, D] = tf2ss(num, denom);



num = [-69950041.16, -2853149735.56, -734760.91];
denom = [57.604, -196510.86, 694116550.287];
c = tf(num, denom);

Gcl = feedback(c*p, 1);

margin(Gcl);

% SP = 2;
% [y, t] = step(SP*Gcl);
% sserror=abs(SP-y(end));