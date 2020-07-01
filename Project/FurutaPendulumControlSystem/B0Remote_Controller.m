%% MECA482 - Furuta Pendulum Controller %%
%This script:
%A) RUNS THE CONTROLLER:
    %1) Runs the controller to swing up and balance the system (C0 steps);
    %2) Brings the system to the theta_d = 0 for C1 steps;
    %3) It follows reference r = theta_d = 1 rad for C2 steps;
    %4) It follows custom reference theta_d for C3 steps;
%B) EXAMINE THE RESPONSE:
%Plot the system response and measure the actual Overshoot and Settling
%Time of the system
clear all;
pause on;
%% Connection to Coppelia
client=b0RemoteApi('b0RemoteApi_matlabClient','b0RemoteApiAddOn');
%% Simulation Setup
%Simulation Step and Duration
%Select here the simulation step time[s]
step_time = 0.005;
% This is a wait time to improve real time and accuracy of the controller.
% This wait_time allow to Coppelia to compute the next step before
% measuring any state from it.
wait_time = step_time/3.5;
%Time[s] to swing_up system
swing_up_time = 10;
%Time[s] to bring the system at theta = 0
regulator_time = 8;
%Time[s] to follow a unitary step
unitarystep_time = 5;
%Time[s] to follow a custom step
customstep_time = 5;
C0 = round(swing_up_time/step_time);
C1 = round(regulator_time/step_time);
C2 = round(unitarystep_time/step_time);
C3 = round(customstep_time/step_time);
simulation_time = swing_up_time+regulator_time+unitarystep_time+customstep_time;
fprintf("The simulation time is going to last %d seconds\n.", simulation_time);
%Setting the Step Time of the simulation
client.simxSetFloatParameter(1, step_time, client.simxServiceCall());
%% Setting up controller (Swing-Up Balance and Input Tracker)
[K,N,info,Ep,Ek,K_swing1, K_swing2, mu, eps] = setup_FURPEN();
%% Setting the output vector
output = zeros(2,C0+C1+C2+C3);
%% Setting Synchronous Communication
%Enable the synchronous mode - Each simulation step is triggered 
client.simxSynchronous(true);
%% Start Simulation
client.simxStartSimulation(client.simxDefaultPublisher());
%% Initializing State
client.simxCallScriptFunction('init_measure_state@Dummy',6,-1,client.simxDefaultPublisher());
client.simxGetSimulationTime(client.simxServiceCall());
client.simxSynchronousTrigger();
pause(wait_time);
%% Part A) - Running controller
%% Step 1) - Running the Swing Up Control and Balance the system
eps_dyn=eps;
E_r = 0;
for i=1:1:C0
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    if (abs(alpha) < eps_dyn)
        r = theta;
        Vm = -K*x+N*r;
        eps_dyn = 0.5;
    else
        E = Ek*alphadot^2+Ep*(1-cos(alpha));
        u = mu*(E - E_r)*sign(alphadot*cos(alpha));
        Vm = K_swing1*u+K_swing2*thetadot;
        eps_dyn = eps;
    end    
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,i) = tt{1,2};
    output(2,i) = theta;
    client.simxSynchronousTrigger();
    %Let the system evolve
    pause(wait_time);
end
client.simxCallScriptFunction('reset_state@Dummy',6,-1,client.simxServiceCall());
client.simxCallScriptFunction('reset_state@Frame',1,-1,client.simxServiceCall());
rr = r;
%% Step 2) - Bring theta to 0
r = 0;
for i=1:1:C1
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    if (abs(alpha) < eps_dyn)
        Vm = -K*x+N*r;
    else
        Vm = 0;
    end
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,C0+i) = tt{1,2};
    output(2,C0+i) = theta;
    client.simxSynchronousTrigger();
    %Let the system evolve
    pause(wait_time);
end
%% Part 3) - Following unitary step
r = 1;
for i=1:1:C2
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    if (abs(alpha) < eps_dyn)
        Vm = -K*x+N*r;
    else
        Vm = 0;
    end
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,C0+C1+i) = tt{1,2};
    output(2,C0+C1+i) = theta;
    client.simxSynchronousTrigger();
    %Let the system evolve
    pause(wait_time);
end
%% Part 4) - Following custom step
r = -3.5;
for i=1:1:C3
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    if (abs(alpha) < eps_dyn)
        Vm = -K*x+N*r;
    else
        Vm = 0;
    end
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,C0+C1+C2+i) = tt{1,2};
    output(2,C0+C1+C2+i) = theta;
    client.simxSynchronousTrigger();
    pause(wait_time); %Let the system evolve
end
%% Stopping Simulation
client.simxStopSimulation(client.simxDefaultPublisher());
client.delete(); 
disp('Simulation ended');
%% Computing Results
%% Plotting system Response
figure(1);
reference = zeros(2,C0+C1+C2+C3);
reference(1,:) = output(1,:);
reference(2,1:C0) = rr;
reference(2,C0+1:C0+C1) = 0;
reference(2,C0+C1+1:C0+C1+C2) = 1;
reference(2,C0+C1+C2+1:C0+C1+C2+C3) = -3.5;
plot(output(1,:),output(2,:));
hold on;
plot(reference(1,:),reference(2,:));
title("Furuta Pendulum System Response");
xlabel("Time");
ylabel("Theta");
legend('SystemResponse','Reference Input');
%% Computing Overshoot and Settling Time
% Overshoot
Peak = max(output(2,C0+C1+1:C0+C1+C2));
perc_overshoot = (Peak-1)/1*100;
% Settling Time
settling_threshold = 0.02;
counter = C0+C1+C2;
while counter > C0 + C1
    if abs(1-output(2,counter)) > settling_threshold
        setling_time=output(1,counter)-output(1,C0+C1+1);
        p = counter;
        counter=0;
    end
    counter = counter-1;
end
disp("Measured Percentage Overshoot:");
disp(perc_overshoot);
disp("Measured Settling Time:");
disp(setling_time);