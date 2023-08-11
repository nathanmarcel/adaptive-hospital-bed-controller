%ME575 Control System Analysis 2 Time responses with transfer functions

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
end;