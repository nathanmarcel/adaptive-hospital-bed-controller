%% Control design via state space methods:

%clear screen, working memory, and close all old figures
clc
clear
close all

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

%% Control design via algebraic methods:
%clear screen, working memory, and close all old figures
clc
clear
close all

num = [0, 0, 0, 608.2];
denom = [0.01736, 62, 2500, 0.7];
p = tf(num, denom);

[A, B, C, D] = tf2ss(num, denom);



num = [-69950041.16, -2853149735.56, -734760.91];
denom = [57.604, -196510.86, 694116550.287];
c = tf(num, denom);

Gcl = feedback(c*p, 1);

margin(Gcl);

%% Control design via frequency domain methods:

%clear screen, working memory, and close all old figures
clc
clear
close all


%set up simulation time in seconds
Tstart=0;
Tend=10;
Tstep=0.001;
time=Tstart:Tstep:Tend;

%set up model parameters for transfer function or state space systems
%plant
numP=[2.236, 6.63e-14];
denP=[1 0 0];

%Uncertainty representation for multiplicative uncertainty 
numDeltaM=[0]; %this will be zero for no uncertainty!
denDeltaM=[1 20 100];

%Uncertainty representation for additive uncertainty
numDeltaA=[0.001];
denDeltaA=[0.01 1]; 


%reference scaling (scalar for SISO systems)
rscale=1;

%reference signal
% unitstep
u1=ones(length(time),1);


%unit ramp
u2=time;

%sinusoidal inputs
u3=sin(3*time)+2*cos(time);

%random inputs from unit normal density
u4=0.1*randn(length(time),1);

%disturbance
d1=1+sin(20*time);
d=timeseries(d1',time');
%system simulation and plotting
for Kgain=[0.1 1 2 5 10 100 1000]
%set up model parameters for controller
numC=[Kgain Kgain];
denC=[1 0];


%Step response
figure(1)
r=timeseries(u1',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('step response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

%ramp response
figure(2)
r=timeseries(u2',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('ramp response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

%sinusoid response
%Step response
figure(3)
r=timeseries(u3',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('optimal control response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

figure(4)
r=timeseries(u4',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('normal noise response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');
end

%% Optimal control design

%clear screen, working memory, and close all old figures
clc
clear
close all

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

%% Testing robustness of designs under uncertainty

%clear screen, working memory, and close all old figures
clc
clear
close all


%set up simulation time in seconds
Tstart=0;
Tend=100;
Tstep=0.001;
time=Tstart:Tstep:Tend;

%set up model parameters for transfer function or state space systems
%plant
numP=[144015.678 2063.744];
denP=[0.01736 62 146515.68 2064.446];

%Uncertainty representation for multiplicative uncertainty 
numDeltaM=[0]; %this will be zero for no uncertainty!
denDeltaM=[1 20 100];

%Uncertainty representation for additive uncertainty
numDeltaA=[0.001];
denDeltaA=[0.01 1]; 


%reference scaling (scalar for SISO systems)
rscale=1;

%reference signal
% unitstep
u1=ones(length(time),1);


%unit ramp
u2=time;

%sinusoidal inputs
u3=sin(3*time)+2*cos(time);

%random inputs from unit normal density
u4=0.1*randn(length(time),1);

%disturbance
d1=1+sin(20*time);
d=timeseries(d1',time');
%system simulation and plotting
for Kgain=[0.1 1 2 5 10 100 1000]
%set up model parameters for controller
numC=[Kgain Kgain];
denC=[1 0];


%Step response
figure(1)
r=timeseries(u1',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('step response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

%ramp response
figure(2)
r=timeseries(u2',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('ramp response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

%sinusoid response
%Step response
figure(3)
r=timeseries(u3',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('optimal control response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');

figure(4)
r=timeseries(u4',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('normal noise response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, seconds');


figure(5)
margin(sys)

figure(6)
nyquist(sys)
end

%% Implementation on high fidelity models

%clear screen, working memory, and close all old figures
clc
clear
close all

%set up simulation time in seconds
Tstart=0;
Tend=100;
Tstep=0.001;
time=Tstart:Tstep:Tend;

%set up model parameters for transfer function or state space systems
%plant
numP=[608.2];
denP=[0.01736 62 2500 0.7];

%Uncertainty representation for multiplicative uncertainty 
numDeltaM=[0]; %this will be zero for no uncertainty!
denDeltaM=[1 20 100];

%Uncertainty representation for additive uncertainty
numDeltaA=[0.001];
denDeltaA=[0.01 1]; 


%reference scaling (scalar for SISO systems)
rscale=1;

%reference signal
% unitstep
u1=ones(length(time),1);


%unit ramp
u2=time;

%sinusoidal inputs
u3=sin(3*time)+2*cos(time);

%random inputs from unit normal density
u4=0.1*randn(length(time),1);

%disturbance
d1=1+sin(20*time);
d=timeseries(d1',time');
%system simulation and plotting
for Kgain=[0.1 1 2 5 10 100 1000]
%set up model parameters for controller
numC=[Kgain Kgain];
denC=[1 0];


%Step response
figure(1)
r=timeseries(u1',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('step response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, minutes');

%ramp response
figure(2)
r=timeseries(u2',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('ramp response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, minutes');

%sinusoid response
%Step response
figure(3)
r=timeseries(u3',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('sinusoidal response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, minutes');

figure(4)
r=timeseries(u4',time');
%run simulink 
simout=sim('ME575ControlAnalysis2sim');
subplot(311);
plot(r, 'b:');hold on
plot(simout.y); ylabel('system output'); title('normal noise response'); xlabel('');
subplot(312);
plot(simout.e); ylabel('tracking error'); title(''); xlabel('');
subplot(313);
plot(simout.u); ylabel('control input'); title('');
xlabel('time, minutes');

end