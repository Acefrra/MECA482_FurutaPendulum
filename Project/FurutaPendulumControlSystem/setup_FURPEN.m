%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   MECA482 - Furuta Pendulum Controller
%   Aaron Taylor-Angel Sanchez-Ingrid Tisell-Michele Fragasso-Joe Karam
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function compute the gain controller K and the scaling block N touse
%the system as a tracker of the reference signal which is the angle 
%theta(t) of the rotating arm
%% SETUP Rotpen
function [K,N,info,Ep, Ek,K_swing1,K_swing2,mu,eps] = setup_FURPEN()
    % Set the electromechanical parameters
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
    m__r = 0.250;
    L__p = 0.335e0;
    J__r = 0.23e-2;
    B__p = 0.0e0;
    tau__1 = 0;
    g = 0.981e1;
    tau__2 = 0;
    B__r = 0.0e0;
    %This runs furuta pendulum model and sets up its state space representation
    FURPEN_SSR_eqns;
    %% Balance controller
    % The requirements of the system
    perc_OS = 10;   %12 Overshoot
    Ts = 3;       %1.5s Settling time
    disp("Desired Overshoot:");
    disp(perc_OS);
    disp("Desired Settling Time:");
    disp(Ts);
    %Specifications
    zeta = (-(log(perc_OS/100))/sqrt(pi^2+(log(perc_OS/100))^2));
    wn = 4/(zeta*Ts);
    sigma = zeta * wn;
    wd = wn*(1-zeta^2)^0.5;
    % Desired poles location
    p1 = -sigma+1i*wd;
    p2 = conj(p1);
    p3 = -sigma*10;
    % One of the excessive poles has a negative real part. So this poles
    % can be cancelled and pass from 4th to 3rd order system.
    p4 = -5.9473;
    % State-feedback gain setup
    K = control_FURPEN(A, B, p1, p2, p3, p4);
    %% Gain Block Input Tracker 
    disp(' ');
    disp('Control gains: ');
    disp(K);
    disp('Closed loop eigenvalues');
    disp(eig(A-B*K));
    [num,den]=ss2tf(A-B*K,B,C,D);
    TF = tf(num,den);
    DC = dcgain(TF);
    %Computing gain
    N = 1/DC;
    info = stepinfo(TF);
    disp("Closed Loop Overshoot");
    disp(info.Overshoot);
    disp("Closed Loop Settling Time");
    disp(info.SettlingTime);
    %% Energy Shaping Controller Setup
    eps = 0.2;
    mu = 25; % You choose this value
    K_swing1 = r__m*m__r*L__r/(eta__g*k__g*eta__m*k__t);
    K_swing2 = k__g*k__m;
    Ek = 1/2*J__p;
    Ep = 1/2*m__p*g*L__p;
end
%% Plot the result