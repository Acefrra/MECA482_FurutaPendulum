function connection_model()
    
    client=b0RemoteApi('b0RemoteApi_matlabClient','b0RemoteApiAddOn');
    
    s2=client.simxGetObjectHandle('Revolute1',client.simxServiceCall());
    %%Starting Simulation
    client.simxStartSimulation(client.simxServiceCall());
    %%Init_state
    client.simxCallScriptFunction('init_measure_state@Dummy',6,-1,client.simxServiceCall());
    for i_counter=1:1:100
        a = client.simxCallScriptFunction('measure_state@Dummy',6,-1,client.simxServiceCall());
        theta = a{1,2}{1};
        alpha = a{1,2}{2};
        thetadot = a{1,2}{3};
        alphadot = a{1,2}{4};
        x = [theta, alplha, thetadot, alphadot];
        %fprintf("%f %f %f %f \n",theta, alpha,thetadot,alphadot);
        %Vm = 3*(thetadr - theta);
        client.simxCallScriptFunction('input_voltage@Frame',1,Vm,client.simxServiceCall());
    end
    client.simxStopSimulation(client.simxServiceCall());
    
    
    client.delete();    
    
    disp('Program ended');
end
