
clc
close all;
tc=cputime;

global J m M e1 e2 e3 Pr La g p ki kp pt kt

h=.01;                                 % Step size
tf=40;                                 % Final time for simulation
t0=0;
e1=[1;0;0];e2=[0;1;0];e3=[0;0;1];
    
% UAV parameters 
J=diag([0.0008,0.0008,0.0014]);         % kgm^2; UAV inertia; 3 x 3 matrix
m=0.8004;                               % kg, mass of UAV
M=m*eye(3);                             % Mass matrix
g=9.81;                                 % Gravity

% Initialization
%  R0=[1 0 0;0 1 0;0 0 1];
R0 = [-0.8487    0        -0.5288;
       0.4197    0.6083   -0.6736;
       0.3217   -0.7937   -0.5163];
b0=[0;0;0];                             % initial position
Om0=[0 0 0]';                           % initial angular velocity
nu0=[0 0 0]';                       
% initial translational velocity
vd0=[0;0;0];                            % initial desired translational velocity
dvd0=[0;0;0];                           % initial desired translational acceleration

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Gains for the sine trajectory
kt = 0.78;%less than 1 always
Pr = eye(3);
pt = 1.2;

% Attitude Gain
La = 0.025*eye(3); % lower the better for trq
p  = 1.15; % 1.1; p>1
ki = 3.4;
kp = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %new gains modified by somesh for sine trajectory for 1200 m distance range
% kt = 0.95;%less than 1 always
% Pr = 40*eye(3);
% pt = 1.2;
% 
% % Attitude Gain
% La = 0.05*eye(3); % lower the better for trq
% p  = 0.98; % 1.1; p>1 %lower the better for initial torque
% ki = 3.35;
% kp = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %gains for cosine trajectory
% kt = 0.8;%less than 1 always
% Pr = 32*eye(3);
% pt = 1.1;
% 
% % Attitude Gain
% La = 0.05*eye(3); % lower the better for trq
% p  = 1.2; % 1.1; p>1
% ki = 3.25;
% kp = 0.01;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[t,R,Rd,b,fm,tau,tau1,tau2,tau3,tau4,nu,Omd,dOmd,Om,Q,bd,vd,dvd,bt,vt] = LGVI_SE3_UAV(b0,R0,nu0,Om0,dvd0,t0,tf,h);

% Plots the results
Results_CUSE(t,bd,b,bt,vt,R,Q,fm,tau,Rd,Omd,dOmd);

