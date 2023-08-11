
num = [0, 0, 0, 608.2];
denom = [0.01736, 62, 2500, 0.7];
sys = tf(num, denom);

[A, B, C, D] = tf2ss(num, denom);

G = ss(A, B, C, D);

sf_poles = [-2989.611; -5.779; -0.001];

[A, B, C, D] = ssdata(G);
K = place(A, B, sf_poles);

sf_ltf = ss(A, B, K, 0);

tf(sf_ltf);
margin(sf_ltf);

ob_poles = [-2989.611; -5.779; -0.001];
L = place(A', C', ob_poles);
L = L';


C_ss = ss(A - B*K-L*C, L, K, 0);
F_ss = ss(A - B*K-L*C, B, -K, -1);

obc_ltf = G*C_ss;
tf(obc_ltf);
margin(obc_ltf);
% t = 0:0.01:2;
% u = zeros(size(t));
% x0 = [0.001 0 0];
% 
% sys = ss(A,B,C,0);
% 
% [y,t,x] = lsim(sys,u,t,x0);
% plot(t,y)
% title('Open-Loop Response to Non-Zero Initial Condition')
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')
% 
% 
% p1 = -10 + 10i;
% p2 = -10 - 10i;
% p3 = -50;
% 
% K = place(A,B,[p1 p2 p3]);
% sys_cl = ss(A-B*K,B,C,0);
% 
% lsim(sys_cl,u,t,x0);
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')
% 
% p1 = -20 + 20i;
% p2 = -20 - 20i;
% p3 = -100;
% 
% K = place(A,B,[p1 p2 p3]);
% sys_cl = ss(A-B*K,B,C,0);
% 
% lsim(sys_cl,u,t,x0);
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')
% 
% 
% t = 0:0.01:2;
% u = 0.001*ones(size(t));
% 
% sys_cl = ss(A-B*K,B,C,0);
% 
% lsim(sys_cl,u,t);
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')
% axis([0 2 -4E-6 0])
% 
% Nbar = rscale(sys,K)
% 
% lsim(sys_cl,Nbar*u,t)
% title('Linear Simulation Results (with Nbar)')
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')
% axis([0 2 0 1.2*10^-3])
% 
% op1 = -100;
% op2 = -101;
% op3 = -102;
% 
% L = place(A',C',[op1 op2 op3])';
% 
% At = [ A-B*K             B*K
%        zeros(size(A))    A-L*C ];
% 

