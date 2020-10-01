%Residual code from experimenting with 'return to home' behaviour in V-REP maximal simulation


vrep = remApi('remoteApi');             %Creates a new remoteAPI object
vrep.simxFinish(-1);                    %Closes any currently open connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);  %Created a new connection

if (clientID>-1)
       disp('Succesfully connected to remote API server');
        %Handle Retrieve
       [returnCode, left_motor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);
       [returnCode, right_motor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking);
       [returnCode, agent] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking);
       [returnCode, front_Sensor] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5', vrep.simx_opmode_blocking);
       
       %Functional Code
       [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_motor, 2, vrep.simx_opmode_blocking); %Set r motor speed
       [returnCode, detectionState, detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);%Read sensor
       [returnCode, startPos] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_streaming); %Log Start Pos
       
       %Random Movement
       for i=1:15
           leftMotorSpeed = 2*rand
           rightMotorSpeed = 2*rand
           [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_motor, leftMotorSpeed, vrep.simx_opmode_blocking);
           [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_motor, rightMotorSpeed, vrep.simx_opmode_blocking);
           %[returnCode, detectionState, detectedPoint,~,~] = vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);%Update sensor reading
           %disp(norm(detectedPoint));
           pause(0.5)
       end
       
       %[number returnCode,array eulerAngles]=simxGetObjectOrientation(number clientID,number objectHandle,number relativeToObjectHandle,number operationMode)%Get Orientation
       [returnCode, endPos] = vrep.simxGetObjectPosition(clientID, agent, -1, vrep.simx_opmode_buffer); %Log end Pos
       [returnCode] = vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_blocking); %Stop r motor
       [returnCode] = vrep.simxSetJointTargetVelocity(clientID, right_motor, 0, vrep.simx_opmode_blocking); %Stop l motor
       
       
       %Calculate True Home Bearing, 1 = x, 2 = y
       if (endPos(1) < startPos(1) && endPos(2) > startPos(2))%TopLeft
           disp("TopLeft")
           x = atan(abs(endPos(1)/abs(endPos(2))));
           returnHomeBearing = pi - x;
           disp("Return Home Bearing: " + returnHomeBearing)
       elseif (endPos(1) > startPos(1) && endPos(2) > startPos(2))%TopRight
           disp("TopRight")
           x = atan(abs(endPos(1)/abs(endPos(2))));
           returnHomeBearing = pi + x;
           disp("Return Home Bearing: " + returnHomeBearing)
       elseif (endPos(1) < startPos(1) && endPos(2) < startPos(2))%BottomLeft
           disp("BottomLeft")
           x = atan(abs(endPos(1)/abs(endPos(2))));
           returnHomeBearing = x;
           disp("Return Home Bearing: " + returnHomeBearing)
       elseif (endPos(1) > startPos(1) && endPos(2) < startPos(2))%BottomRight
           disp("BottomRight")
           x = atan(abs(endPos(1)/abs(endPos(2))));
           returnHomeBearing = pi*2 -x;
           disp("Return Home Bearing: " + returnHomeBearing)
       end
       
       
       vrep.simxFinish(-1);
end

vrep.delete(); % call the destructor!
disp('Program terminated');
