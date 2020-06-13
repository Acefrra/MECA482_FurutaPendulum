clear all;
K = setup_FURPEN();
client=b0RemoteApi('b0RemoteApi_matlabClient','b0RemoteApiAddOn');
client.simxStartSimulation(client.simxDefaultPublisher());
%%Initializing State
client.simxCallScriptFunction('init_measure_state@Dummy',6,-1,client.simxDefaultPublisher());
client.simxSynchronous(True);

while(toc < 10)%Ir run the simulation for 10s
    a = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
    theta = a{1,2}{1};
    alpha = a{1,2}{2};
    thetadot = a{1,2}{3};
    alphadot = a{1,2}{4};
    x = [theta; alpha; thetadot; alphadot];
    Vm = -K*x;
    client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxDefaultPublisher());
end
client.simxStopSimulation(client.simxDefaultPublisher());
client.delete();    
disp('Simulation ended');
