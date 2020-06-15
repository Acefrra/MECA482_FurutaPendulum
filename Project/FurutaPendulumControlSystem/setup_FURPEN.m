%MECA482 - Furuta Pendulum Controller - Aaron Taylor Angel Sanchez Ingrid
%Tisell Michele Fragasso Joe Karam
%This function compute the gain controller K and the scaling block N to use
%the system as a tracker of the reference signal which is the angle
%theta(t) of the rotating arm.s
%% SETUP Rotpen
function [K,N,info] = setup_FURPEN()
%% Sets the necessary parameters to run
%clear all;
%% Lab Configuration
% Type of controller set it to 
% Pass for now later add the debug mode here
% No Lab
%% Servo Configuration
% Add electromechanical properties of the motor here.
%% Load the model of the system
% Set the electromechanical system model
eta__g = 0.85e0;
eta__m = 0.87e0;
k__g = 70;
k__m = 0.76e-2;
k__t = 0.76e-2;
r__m = 0.26e1;
V__m = 0;
% Set the rotary arm parameters
J__p = 0.23e-2;
m__p = 0.125e0;
L__r = 0.215e0;
m__r = 0;
L__p = 0.335e0;
J__r = 0.23e-2;
B__p = 0.0e0;
tau__1 = 0;
g = 0.981e1;
tau__2 = 0;
B__r = 0.0e0;
% Load pendulum parameters
FURPEN_SSR_eqns; %This runs furuta pendulum model and sets up its state space representation
%% Add controller
% Balance control (small angle approx.)
% epsilon = RAD2DEG*epsilon;
% The requirements of the system
perc_OS = 12;   %12 Overshoot
Ts = 3;       %1.5s Settling time
disp("Desired Overshoot:");
disp(perc_OS);
disp("Desired Settling Time:");
disp(Ts);
zeta = (-(log(perc_OS/100))/sqrt(pi^2+(log(perc_OS/100))^2));
wn = 4/(zeta*Ts);
sigma = zeta * wn;
wd = wn*(1-zeta^2)^0.5;
p1 = -sigma+1i*wd;
p2 = conj(p1);
p3 = -sigma*10; % Desired pole location
%p4 = -sigma*11; % Desired pole location
p4 = -5.9473;
% State-feedback gain
% One can customize this function or uses
% built-in functions of MATLAB
K = control_FURPEN(A, B, p1, p2, p3, p4);
%% Swing-up Control
% Use energy based solution from [Underactuated Robotics, MIT]
%[Er, a_max] = swing_up_FURPEN(eta_m, eta_g, Kg, kt, Rm, Mr, Lr);
%% Display
disp(' ');
disp('Control gains: ');
disp(K);
disp('Closed loop eigenvalues');
disp(eig(A-B*K));
%disp('Swing-up Parameters: ');
%disp( [' Er =', num2str(Er) ' J']);
%disp( [' a_max=' num2str(a_max) 'm/s^2']);
%end
[num,den]=ss2tf(A-B*K,B,C,D);
TF = tf(num,den);
DC = dcgain(TF);
N = 1/DC;
info = stepinfo(TF);
disp("Closed Loop Overshoot");
disp(info.Overshoot);
disp("Closed Loop Settling Time");
disp(info.SettlingTime);
end
%% Plot the result