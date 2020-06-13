function testing()
    function callb(msg)
        disp(jsonencode(msg));
    end
    
    client=b0RemoteApi('b0RemoteApi_matlabClient','b0RemoteApiAddOn');
    
    %s1=client.simxGetObjectHandle('Frame',client.simxServiceCall())
    s2=client.simxGetObjectHandle('Revolute1',client.simxServiceCall());
    %prox=client.simxGetObjectHandle('Revolute2',client.simxServiceCall());
    %vis=client.simxGetObjectHandle('RotatingArm',client.simxServiceCall());
    %fs=client.simxGetObjectHandle('Pendulum',client.simxServiceCall());
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
    
    %fprintf("%d", res{1,1});
    %{        
    client.simxAddStatusbarMessage('Hello',client.simxDefaultPublisher());
    res=client.simxAuxiliaryConsoleOpen('theTitle',50,4,{10,400},{1024,100},{1,1,0},{0,0,0},client.simxServiceCall());
    client.simxAuxiliaryConsolePrint(res{2},'Hello World!!!\\n',client.simxServiceCall());
    client.simxSleep(1000);
    client.simxAuxiliaryConsoleShow(res{2},false,client.simxServiceCall());
    client.simxSleep(1000);
    client.simxAuxiliaryConsoleShow(res{2},true,client.simxServiceCall());
    client.simxSleep(1000);
    client.simxAuxiliaryConsoleClose(res{2},client.simxServiceCall());
    client.simxStartSimulation(client.simxServiceCall());
    client.simxStopSimulation(client.simxServiceCall());
    res=client.simxGetObjectHandle('shape1',client.simxServiceCall());
    client.simxGetObjectPosition(res{2},-1,client.simxDefaultSubscriber(@callb));
    for i=1:100
        client.simxSpinOnce();
        client.simxSleep(100);
    end
    %}

    %{        
    res=client.simxAddDrawingObject_points(8,{255,0,255},{0,0,0,1,0,0,0,0,1},client.simxServiceCall());
    client.simxSleep(1000);
    client.simxRemoveDrawingObject(res{2},client.simxServiceCall());
    res=client.simxAddDrawingObject_spheres(0.05,{255,0,0},{0,0,0,1,0,0,0,0,1},client.simxServiceCall());
    client.simxSleep(1000);
    client.simxRemoveDrawingObject(res{2},client.simxServiceCall());
    res=client.simxAddDrawingObject_cubes(0.05,{255,0,0},{0,0,0,1,0,0,0,0,1},client.simxServiceCall());
    client.simxSleep(1000);
    client.simxRemoveDrawingObject(res{2},client.simxServiceCall());
    res=client.simxAddDrawingObject_segments(4,{0,255,0},{0,0,0,1,0,0, 1,0,0,0,0,1, 0,0,1,0,0,0},client.simxServiceCall());
    client.simxSleep(1000);
    client.simxRemoveDrawingObject(res{2},client.simxServiceCall());
    res=client.simxAddDrawingObject_triangles({255,128,0},{0,0,0, 1,0,0, 0,0,1},client.simxServiceCall());
    client.simxSleep(1000);
    client.simxRemoveDrawingObject(res{2},client.simxServiceCall());
    %}

    %{        
    client.simxStartSimulation(client.simxServiceCall());
    disp(jsonencode(client.simxCheckCollision(s1{2},s2{2},client.simxServiceCall())));
    disp(jsonencode(client.simxCheckDistance(s1{2},s2{2},0,client.simxServiceCall())));
    disp(jsonencode(client.simxCheckProximitySensor(prox{2},s2{2},client.simxServiceCall())));
    disp(jsonencode(client.simxCheckVisionSensor(vis{2},s2{2},client.simxServiceCall())));
    coll=client.simxGetCollisionHandle('coll',client.simxServiceCall());
    dist=client.simxGetDistanceHandle('dist',client.simxServiceCall());
    disp(jsonencode(client.simxReadCollision(coll{2},client.simxServiceCall())));
    disp(jsonencode(client.simxReadDistance(dist{2},client.simxServiceCall())));
    disp(jsonencode(client.simxReadProximitySensor(prox{2},client.simxServiceCall())));
    disp(jsonencode(client.simxReadVisionSensor(vis{2},client.simxServiceCall())));
    disp(jsonencode(client.simxReadForceSensor(fs{2},client.simxServiceCall())));
    disp(jsonencode(client.simxBreakForceSensor(fs{2},client.simxServiceCall())));
    client.simxSleep(1000);
    client.simxStopSimulation(client.simxServiceCall());
    %}
    %{        
    client.simxSetFloatSignal('floatSignal',123.456,client.simxServiceCall());
    client.simxSetIntegerSignal('integerSignal',59,client.simxServiceCall());
    client.simxSetStringSignal('stringSignal','Hello World',client.simxServiceCall());
    disp(jsonencode(client.simxGetFloatSignal('floatSignal',client.simxServiceCall())));
    disp(jsonencode(client.simxGetIntegerSignal('integerSignal',client.simxServiceCall())));
    disp(jsonencode(client.simxGetStringSignal('stringSignal',client.simxServiceCall())));
    client.simxSleep(1000);
    client.simxClearFloatSignal('floatSignal',client.simxServiceCall());
    client.simxClearIntegerSignal('integerSignal',client.simxServiceCall());
    client.simxClearStringSignal('stringSignal',client.simxServiceCall());
    client.simxSleep(1000);
    disp(jsonencode(client.simxGetFloatSignal('floatSignal',client.simxServiceCall())));
    disp(jsonencode(client.simxGetIntegerSignal('integerSignal',client.simxServiceCall())));
    disp(jsonencode(client.simxGetStringSignal('stringSignal',client.simxServiceCall())));

    client.simxCheckProximitySensor(prox{2},s2{2},client.simxDefaultSubscriber(@callb));
    startTime=client.simxGetTimeInMs();
    while (client.simxGetTimeInMs()<startTime+5000) 
        client.simxSpinOnce();
    end
    %}
   %{ 
    disp(jsonencode(client.simxSetObjectPosition(s1{2},-1,[0,0,0.2],client.simxServiceCall())));
    client.simxSleep(1000);
    disp(jsonencode(client.simxSetObjectOrientation(s1{2},-1,[0,0,0.2],client.simxServiceCall())));
    disp(jsonencode(client.simxGetObjectOrientation(s1{2},-1,client.simxServiceCall())));
    client.simxSleep(1000);
    disp(jsonencode(client.simxSetObjectQuaternion(s1{2},-1,[0,0,0.2,1],client.simxServiceCall())));
    disp(jsonencode(client.simxGetObjectQuaternion(s1{2},-1,client.simxServiceCall())));
    client.simxSleep(1000);
    disp(jsonencode(client.simxSetObjectPose(s1{2},-1,[0.1,0.1,0,0,0,0,1],client.simxServiceCall())));
    disp(jsonencode(client.simxGetObjectPose(s1{2},-1,client.simxServiceCall())));
    client.simxSleep(1000);
    matr=client.simxGetObjectMatrix(s1{2},-1,client.simxServiceCall());
    disp(jsonencode(matr));
    matr{2}{4}=0;
    matr{2}{8}=0;
    disp(jsonencode(client.simxSetObjectMatrix(s1{2},-1,matr{2},client.simxServiceCall())));
%}


    %disp(jsonencode(client.simxCallScriptFunction('myFunction@DefaultCamera','sim.scripttype_customizationscript',{'Hello World :)',{255,0,255},[],57.59},client.simxServiceCall())));
    %disp(jsonencode(client.simxCallScriptFunction('myFunction@DefaultCamera','sim.scripttype_customizationscript','Hello World :)',client.simxServiceCall())));
    %disp(jsonencode(client.simxCallScriptFunction('myFunction@DefaultCamera','sim.scripttype_customizationscript',59,client.simxServiceCall())));
    %disp(jsonencode(client.simxCallScriptFunction('myFunction@DefaultCamera','sim.scripttype_customizationscript',{[]},client.simxServiceCall())));

    client.delete();    
    
    disp('Program ended');
end
