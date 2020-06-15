%MECA482 - Furuta Pendulum Controller - Aaron Taylor Angel Sanchez Ingrid
%Tisell Michele Fragasso Joe Karam
%This script:
%1) Runs the controller as a regulator for C1 cycle.
%2) It follows reference r = theta = 1 rad for C2 cycles
%3) It follows reference r = theta = -2 rad
%4) Plot the system response and measure the actual Overshoot and Settling
%Time of the system

clear all;
[K,N,info] = setup_FURPEN();
C1 = 400; %Step to run as regulator
C2 = 300;
C3 = 300;
output = zeros(2,C1+C2+C3);



client=b0RemoteApi('b0RemoteApi_matlabClient','b0RemoteApiAddOn');
client.simxStartSimulation(client.simxDefaultPublisher());
%%Initializing State
client.simxCallScriptFunction('init_measure_state@Dummy',6,-1,client.simxDefaultPublisher());
client.simxGetSimulationTime(client.simxServiceCall())
for i=1:1:C1
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    Vm = -K*x;
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,i) = tt{1,2};
    output(2,i) = theta;
end
r = 1;
for i=1:1:C2
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    Vm = -K*x+N*r;
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,C1+i) = tt{1,2};
    output(2,C1+i) = theta;
end
r = -3.14;
for i=1:1:C3
    state = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = state{1,2}{1};
    alpha = state{1,2}{2};
    thetadot = state{1,2}{3};
    alphadot = state{1,2}{4};
    x = double([theta; alpha; thetadot; alphadot]);
    Vm = -K*x+N*r;
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
    tt = client.simxGetSimulationTime(client.simxServiceCall());
    output(1,C1+C2+i) = tt{1,2};
    output(2,C1+C2+i) = theta;
end

client.simxStopSimulation(client.simxDefaultPublisher());
client.delete(); 

disp('Simulation ended');
figure(1);
reference = zeros(2,C1+C2+C3);
reference(1,:) = output(1,:);
reference(2,1:C1) = 0;
reference(2,C1+1:C1+C2) = 1;
reference(2,C1+C2+1:C1+C2+C3) = -2;
plot(output(1,:),output(2,:));
hold on;
plot(reference(1,:),reference(2,:));
title("Furuta Pendulum System Response");
xlabel("Time");
ylabel("Theta");
legend('SystemResponse','Reference Input');
%Computing Overshoot and Settling Time
Peak = max(output(2,C1+1:C1+C2));
perc_overshoot = (Peak-1)/1*100;
%Computing Settling Time
settling_threshold = 0.02;
counter = C1+C2;
while counter > C1
    if abs(1-output(2,counter)) > settling_threshold
        setling_time=output(1,counter)-output(1,C1+1);
        p = counter;
        counter=0;
    end
    counter = counter-1;
end
disp("Measured Percentage Overshoot:");
disp(perc_overshoot);
disp("Measured Settling Time:");
disp(setling_time);