clear
clc

%Arduino Setup
a = arduino();
writePWMDutyCycle(a,'D6',0.8); %base rotation driver
writePWMDutyCycle(a,'D5',0.8); %shoulder driver
writePWMDutyCycle(a,'D4',0.8); %elbow driver
writePWMDutyCycle(a,'D3',0.8); %wrist driver
writePWMDutyCycle(a,'D2',0.8); %gripper driver

%Draw an image on the screen

x=100;
y=100;
fprintf('x = %f\t\t',x)
fprintf('y = %f\n',y)
[baseAngle,shoulderAngle,elbowAngle,wristAngle] = getAngles(x,y);
goToLocation(a,baseAngle,shoulderAngle,elbowAngle,wristAngle)

%{
for i = 1:10
    shoulderUp(a)
    wristDown(a);
end
%}
function [baseAngleV,shoulderAngleV,elbowAngleV,wristAngleV] = getAngles(x,y)
    %OWI Arm Constants in mm.
    BASE_HEIGHT = 70;
    L1 = 90;
    L2 = 110;
    L3 = 110;
    z = 200; %constant gripper height
    VOLT_PER_DEG = 0.019;
    MAX_EXT_LEN = L1+L2+L3;
    EXT_LEN = sqrt(x*x + y*y);
    
    if EXT_LEN > MAX_EXT_LEN
        fprintf('Extension length exceeds maximum reach.\n')
        return;
    end
    
    baseAngle = atan2d(y,x);
    baseAngleV = 4.21 - baseAngle*VOLT_PER_DEG;
    fprintf('baseAngle = %f\t\t',baseAngle)
    fprintf('baseAngleV = %f\n',baseAngleV)
    radialDist = sqrt(x*x + y*y);
    wristZ = z - BASE_HEIGHT;
    wristY = radialDist - L3;
    swDist = sqrt(wristZ*wristZ + wristY*wristY);
    swAngle1 = atan2d(wristZ,wristY);
    swAngle2 = acosd((L1*L1 + swDist*swDist - L2*L2)/(2*L1*swDist));
    fprintf('swAngle1 = %f\t\t',swAngle1);
    fprintf('swAngle2 = %f\n',swAngle2);
    %shoulderAngle = 90 - (swAngle1 + swAngle2);
    shoulderAngleAbs = abs(180 - swAngle1 - swAngle2);
    shoulderAngleV = shoulderAngleAbs*VOLT_PER_DEG + 0.79;
    fprintf('shoulderAngle = %f\t',shoulderAngleAbs)
    fprintf('shoulderAngleV = %f\n',shoulderAngleV)
    ewAngle = acosd((L1*L1 + L2*L2 - swDist*swDist)/(2*L1*L2));
    elbowAngle = 90 - ewAngle;
    elbowAngleV = elbowAngle*VOLT_PER_DEG + 0.79;
    fprintf('elbowAngle = %f\t\t',elbowAngle)
    fprintf('elbowAngleV = %f\n',elbowAngleV)
    wristAngle = abs(90 - (shoulderAngleAbs + elbowAngle));
    wristAngleV = wristAngle*VOLT_PER_DEG + 0.79;
    %wristAngleV = 4.21 - wristAngle*VOLT_PER_DEG;
    fprintf('wristAngle = %f\t\t',wristAngle)
    fprintf('wristAngleV = %f\n',wristAngleV)
    return
end
%Go To Location
function goToLocation(ard,base,shoulder,elbow,wrist)
    locationReached = 0
    baseAngleReached = 0;
    shoulderAngleReached = 0;
    elbowAngleReached = 0;
    wristAngleReached = 0;
    oldBase = readVoltage(ard,'A5');
    %fprintf('oldBase = %f\t', oldBase)
    oldShoulder = readVoltage(ard,'A4');
    oldElbow = readVoltage(ard,'A3');
    oldWrist = readVoltage(ard,'A2');
    while locationReached == 0
        currentBase = readVoltage(ard,'A5');
        fprintf('targetBase = %f\t', base)
        fprintf('currentBase = %f\n', currentBase)
        currentShoulder = readVoltage(ard,'A4');
        fprintf('targetShoulder = %f\t', shoulder)
        fprintf('currentShoulder = %f\n', currentShoulder)
        currentElbow = readVoltage(ard,'A3');
        fprintf('targetElbow = %f\t', elbow)
        fprintf('currentElbow = %f\n', currentElbow)
        currentWrist = readVoltage(ard,'A2');
        fprintf('targetWrist = %f\t', wrist)
        fprintf('currentWrist = %f\n', currentWrist)
        if baseAngleReached == 1 
            stopBase(ard)
        elseif currentBase < base + 0.01
            baseNeg(ard)
        elseif currentBase > base - 0.01
            basePos(ard)
        end
        if shoulderAngleReached == 1
            stopShoulder(ard)
        elseif currentShoulder > shoulder + 0.01
            shoulderDown(ard)
        elseif currentShoulder < shoulder - 0.01
            shoulderUp(ard)
        end
        
        if elbowAngleReached == 1
            stopElbow(ard)
        elseif currentElbow > elbow + 0.01
            elbowDown(ard)
        elseif currentElbow < elbow - 0.01
            elbowUp(ard)
        end
        
        if wristAngleReached == 1
            stopWrist(ard)
        elseif currentWrist > wrist + 0.01
            wristUp(ard)
        elseif currentWrist < wrist - 0.01
            wristDown(ard)
        end
        
        if currentBase >= base - 0.1 && currentBase <= base + 0.1
            baseAngleReached = 1
            stopBase(ard)
        end
        if currentShoulder >= shoulder - 0.1 && currentShoulder <= shoulder + 0.1
            shoulderAngleReached = 1
            stopShoulder(ard)
        end
        if currentElbow >= elbow - 0.1 && currentElbow <= elbow + 0.1
            elbowAngleReached = 1
            stopElbow(ard)
        end
        if currentWrist >= wrist - 0.1 && currentWrist <= wrist + 0.1
            wristAngleReached = 1
            stopWrist(ard)
        end
        if baseAngleReached == 1 && shoulderAngleReached == 1 && elbowAngleReached == 1 && wristAngleReached == 1
            locationReached = 1
            stopBase(ard)
            stopShoulder(ard)
            stopElbow(ard)
            stopWrist(ard)
        end
    end
end
%Gripper
function gripperIn(ard)
    writeDigitalPin(ard,'D22',1);
    writeDigitalPin(ard,'D24',0);
end 
function gripperOut(ard)
    writeDigitalPin(ard,'D22',0);
    writeDigitalPin(ard,'D24',1);
end
%Wrist Angle
function wristUp(ard)
    writeDigitalPin(ard,'D26',1);
    writeDigitalPin(ard,'D28',0);
end
function wristDown(ard)
    writeDigitalPin(ard,'D26',0);
    writeDigitalPin(ard,'D28',1);
end 
%Elbow Angle
function elbowUp(ard)
    writeDigitalPin(ard,'D30',1);
    writeDigitalPin(ard,'D32',0);
end 
function elbowDown(ard)
    writeDigitalPin(ard,'D30',0);
    writeDigitalPin(ard,'D32',1);
end 
%Base Angle
function shoulderUp(ard)
    writeDigitalPin(ard,'D34',0);
    writeDigitalPin(ard,'D36',1);
end function shoulderDown(ard)
    writeDigitalPin(ard,'D34',1);
    writeDigitalPin(ard,'D36',0);
end 
%Base Rotation
function basePos(ard)
    writeDigitalPin(ard,'D38',1);
    writeDigitalPin(ard,'D40',0);
end
function baseNeg(ard)
    writeDigitalPin(ard,'D38',0);
    writeDigitalPin(ard,'D40',1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Stop Movements
function stopGripper(ard)
    writeDigitalPin(ard,'D22',0);
    writeDigitalPin(ard,'D24',0);
end
function stopWrist(ard)
    writeDigitalPin(ard,'D26',0);
    writeDigitalPin(ard,'D28',0);
end
function stopElbow(ard)
    writeDigitalPin(ard,'D30',0);
    writeDigitalPin(ard,'D32',0);
end
function stopShoulder(ard)
    writeDigitalPin(ard,'D34',0);
    writeDigitalPin(ard,'D36',0);
end
function stopBase(ard)
    writeDigitalPin(ard,'D38',0);
    writeDigitalPin(ard,'D40',0);
end