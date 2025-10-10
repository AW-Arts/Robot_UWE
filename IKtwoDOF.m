%%%%%%%%%%%%% Antonia Tzemanaki %%%%%%%%%%%
%%%%%%%%%%%%%% Robotic Systems %%%%%%%%%%%%
%%%%%%%%%%%%%%%% October 2017 %%%%%%%%%%%%%

clear all
close all
clc

%% Links Length
l1 = 0.1 ;
l2 = 0.05 ;

%% Desired position of end-effector
px = 0.035355 ;
py = 0.135355 ;
pz = 0 ;

p0e = [ px py pz ]' ;
disp('Desired position =')
disp(p0e)
if norm(p0e) > l1+l2
    error('desired position is out of the workspace')
end

%% %%%%%%%%%%%% Inverse Kinematics of a 2DOF manipulator %%%%%%%%%%%%%%%%%

%% Find q2
c2 = (px^2+py^2-l1^2-l2^2)/(2*l1*l2) 
s21 = sqrt(1-c2^2) ;
s22 = - sqrt(1-c2^2) ;
% We have two solutions for q2
q21 = atan2(s21,c2) 
q22 = atan2(s22,c2) 


%% Find q1

% We define the constants
k1 = l1+(l2*c2) ;
k21 = l2*s21 ;
k22 = l2*s22 ;
gama1 = atan2(k21,k1) ;
gama2 = atan2(k22,k1) ;
r = sqrt(k1^2+k21^2) ; % r is the same for both values of q2

% We find q1 depending on the value of q2
q11 = atan2(py/r,px/r) - gama1 ;
q12 = atan2(py/r,px/r) - gama2 ;


disp('1st solution')
disp([ rad2deg(q11) rad2deg(q21) ])

disp('2nd solution')
disp([ q12 q22 ]*180/pi)


%% %%%%%%%%%%%%%%%%%%%%% IK Using Robotic Toolbox %%%%%%%%%%%%%%%%%%%%%%%%%
disp('%% %%%%%%%%%%%%%%%%%%%%% IK Using Robotic Toolbox %%%%%%%%%%%%%%%%%%%%%%%%%')
%% Standard DH parameters

L1 = Link('a',l1,'alpha',0,'d',0);
L2 = Link('a',l2,'alpha',0,'d',0);

RR = SerialLink([L1 L2])

%% Define the EE HT
Transl = transl(px,py,pz)

Rot = rpy2tr([0 0 45],'deg')

T = Transl*Rot

%% IK - numerical 1

Q_num = RR.ikine(T,[0 0],[1 1 0 0 0 0]);
rad2deg(Q_num)
RR.plot(Q_num)

%% IK - numerical 2

Q_num2 = RR.ikine(T,[0.1 0.1],[1 1 0 0 0 0]);
rad2deg(Q_num2)
RR.plot(Q_num2)

%% IK - optimisation

[Q_opt,err] = RR.ikunc(T) 
rad2deg(Q_opt)
RR.plot(Q_opt)