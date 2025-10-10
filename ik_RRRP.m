%% RRRP Model
clear all
clc
clf

% You need the legths l1, l2, l3, l4 and the DH parameters table.
% Then you have for each link:
L1 = Link('d', 1,'a',2,'alpha',0); % Where d = l1 =1, a=l2 = 2
L2 = Link('d', 0,'a',1,'alpha',pi); % Where a = l3 =1, alpha = pi
L3 = Link('d', 0.5,'a',0,'alpha',0); % Where d = l4 =1
L4 = Prismatic('theta',0,'a',0,'alpha',0); % It is prismatic

% Define the serially connected robot:
RRRP = SerialLink([L1 L2 L3 L4])

% You need to set limits for the prismatic
RRRP.qlim(4,1)=0;
RRRP.qlim(4,2)=1;

%You can now plot the robot, due to the prismatic you need to use the
%option 'workspace'
RRRP.plot([0 0 0 0],'workspace',[-6 6 -6 6 -4 4])

%% Define the End Effector HT

T = [0.5736 0.8192 0 0.995;
     0.8192 -0.5736 0 0.983;
     0 0 -1 0.25;
     0 0 0 1];
 
 % Normalise the rotation matrix
 T = trnorm(T)
 
 
%% IK - Optimisation

Q_opt = RRRP.ikunc(T)
RRRP.plot(Q_opt,'workspace',[-6 6 -6 6 -4 4])

%% IK - Numerically 1

qinit = [deg2rad([0 0 0]) 0];
Q_num = RRRP.ikine(T,qinit,[1 1 1 0 0 1])
RRRP.plot(Q_num,'workspace',[-6 6 -6 6 -4 4])

%% IK - Numerically 2

qinit = [deg2rad([0 50 0]) 0];
Q_num2 = RRRP.ikine(T,qinit,[1 1 1 0 0 1])
RRRP.plot(Q_num2,'workspace',[-6 6 -6 6 -4 4])

%% Trajectory - THIS IS EXTRA WILL SEE IT IN A COUPLE OF WEEKS
% Do a multi-axis trajectory
qtr = mtraj(@tpoly,Q_opt,Q_num2,50);
% Plot it
RRRP.plot(qtr,'workspace',[-6 6 -6 6 -4 4])
