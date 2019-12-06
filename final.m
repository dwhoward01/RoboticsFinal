clear
clc
%Arduino Setup
a = arduino();
writePWMDutyCycle(a,'D6',1); %base rotation driver
writePWMDutyCycle(a,'D5',1); %shoulder driver
writePWMDutyCycle(a,'D4',1); %elbow driver
writePWMDutyCycle(a,'D3',1); %wrist driver
writePWMDutyCycle(a,'D2',1); %gripper driver
%Draw an image on the screen
fontSize = 16;
axis on;
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
message = sprintf('Left click and hold to begin drawing a freehand path.\nDraw within (0.5,0.5). Lift the mouse button to finish.');
uiwait(msgbox(message));
% User draws curve on image here.
hFH = imfreehand();
% Get the xy coordinates of where they drew.
xy = hFH.getPosition;
% get rid of imfreehand remnant.
delete(hFH);
% Overlay what they drew onto the image.
%hold on; % Keep image, and direction of y axis.
xCoordinates = xy(:, 1);
yCoordinates = xy(:, 2);
plot(xCoordinates, yCoordinates, 'ro', 'LineWidth', 2, 'MarkerSize', 10);
xlim([0 0.5]);
ylim([0 0.5]);
%Robot recreates drawing
for i = 1:length(xy)
    fprintf('i = %d\t', i)
    x = xy(i,1)*100+100;
    y = xy(i,2)*100+100;
    fprintf('x: %f\t', x)
    fprintf('y: %f\n', x)
    
    [baseAngle,shoulderAngle,elbowAngle,wristAngle] = getAngles(x,y);
    [locationReached] = goToLocation(a,baseAngle,shoulderAngle,elbowAngle,wristAngle);
end
function [theta1V,theta2V,theta3V,theta4V] = getAngles(x,y)
    %OWI Arm Constants in mm.
    L1 = 90;
    L2 = 110;
    VOLT_PER_DEG = 0.019;
    MAX_EXT_LEN = L1+L2;
    EXT_LEN = sqrt(x*x + y*y);
    if EXT_LEN > MAX_EXT_LEN
        msg = 'Extension length exceeds maximum reach.';
        error(msg)
        return;
    end
    theta1 = atan2d(y,x);
    theta1V = theta1*VOLT_PER_DEG + 0.4936;
    fprintf('theta1 = %f\t\t',theta1)
    fprintf('theta1V = %f\n',theta1V)
    r = sqrt(x*x + y*y);
    theta2 = acosd((L1*L1 + r*r - L2*L2)/(2*L1*r));
    theta2V = theta2*VOLT_PER_DEG + 0.79;
    fprintf('theta2 = %f\t\t',theta2)
    fprintf('theta2V = %f\n',theta2V)
    theta3P = acosd((L1*L1 + L2*L2 - r*r)/(2*L1*L2));
    theta3PV = theta3P*VOLT_PER_DEG;
    theta3 = 180 - theta3P;
    theta3V = 2.5 - theta3*VOLT_PER_DEG;
    fprintf('theta3P = %f\t\t',theta3P)
    fprintf('theta3PV = %f\n',theta3PV)
    fprintf('theta3 = %f\t\t',theta3)
    fprintf('theta3V = %f\n',theta3V)
    theta4 = theta3 - theta2;
    theta4V = 2 - theta4*VOLT_PER_DEG;
    fprintf('theta4 = %f\t\t',theta4)
    fprintf('theta4V = %f\n',theta4V)
    
    return
end
%Go To Location
function [locationReached] = goToLocation(ard,base,shoulder,elbow,wrist)
    locationReached = 0;
    baseAngleReached = 0;
    shoulderAngleReached = 0;
    elbowAngleReached = 0;
    wristAngleReached = 0;
    while locationReached == 0
        %Debug lines
        %{
        currentBase = readVoltage(ard,'A5');
        fprintf('targetBase = %f\t\t', base)
        fprintf('currentBase = %f\n', currentBase)
        currentShoulder = readVoltage(ard,'A4');
        fprintf('targetShoulder = %f\t', shoulder)
        fprintf('currentShoulder = %f\n', currentShoulder)
        currentElbow = readVoltage(ard,'A3');
        fprintf('targetElbow = %f\t\t', elbow)
        fprintf('currentElbow = %f\n', currentElbow)
        currentWrist = readVoltage(ard,'A2');
        fprintf('targetWrist = %f\t\t', wrist)
        fprintf('currentWrist = %f\n', currentWrist)
        fprintf('*******************************************************\n')
        %}
        if baseAngleReached == 1 
            stopBase(ard)
        elseif currentBase > base + 0.01
            baseNeg(ard)
        elseif currentBase < base - 0.01
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
            baseAngleReached = 1;
            fprintf('Base Angle Reached\n')
            stopBase(ard)
        end
        if currentShoulder >= shoulder - 0.1 && currentShoulder <= shoulder + 0.1
            shoulderAngleReached = 1;
            fprintf('Shoulder Angle Reached\n')
            stopShoulder(ard)
        end
        if currentElbow >= elbow - 0.1 && currentElbow <= elbow + 0.1
            elbowAngleReached = 1;
            fprintf('Elbow Angle Reached\n')
            stopElbow(ard)
        end
        if currentWrist >= wrist - 0.1 && currentWrist <= wrist + 0.1
            wristAngleReached = 1;
            fprintf('Wrist Angle Reached\n')
            stopWrist(ard)
        end
        if baseAngleReached == 1 && shoulderAngleReached == 1 && elbowAngleReached == 1 && wristAngleReached == 1
            locationReached = 1;
            fprintf('Base Angle Reached\n')
            stopBase(ard)
            stopShoulder(ard)
            stopElbow(ard)
            stopWrist(ard)
        end
    end
end
%Joint Direction Functions
%Gripper
function gripperIn(ard)
    writeDigitalPin(ard,'D22',1);
    writeDigitalPin(ard,'D24',0);
end 
function gripperOut(ard)
    writeDigitalPin(ard,'D22',0);
    writeDigitalPin(ard,'D24',1);
end
%Wrist
function wristUp(ard)
    writeDigitalPin(ard,'D26',1);
    writeDigitalPin(ard,'D28',0);
end
function wristDown(ard)
    writeDigitalPin(ard,'D26',0);
    writeDigitalPin(ard,'D28',1);
end 
%Elbow
function elbowUp(ard)
    writeDigitalPin(ard,'D30',1);
    writeDigitalPin(ard,'D32',0);
end 
function elbowDown(ard)
    writeDigitalPin(ard,'D30',0);
    writeDigitalPin(ard,'D32',1);
end 
%Shoulder
function shoulderUp(ard)
    writeDigitalPin(ard,'D34',0);
    writeDigitalPin(ard,'D36',1);
end function shoulderDown(ard)
    writeDigitalPin(ard,'D34',1);
    writeDigitalPin(ard,'D36',0);
end 
%Base
function basePos(ard)
    writeDigitalPin(ard,'D38',1);
    writeDigitalPin(ard,'D40',0);
end
function baseNeg(ard)
    writeDigitalPin(ard,'D38',0);
    writeDigitalPin(ard,'D40',1);
end
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