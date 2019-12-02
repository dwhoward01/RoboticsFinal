clc;  % Clear command window.
clear;  % Delete all variables.
close all;  % Close all figure windows except those created by imtool.
imtool close all;  % Close all figure windows created by imtool.
workspace;  % Make sure the workspace panel is showing.
fontSize = 16;
axis on;
set(gcf, 'Position', get(0,'Screensize')); % Maximize figure.
message = sprintf('Left click and hold to begin drawing a freehand path.\nLift the mouse button to finish.');
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
xlim([0 1]);
ylim([0 1]);

VOLT_PER_DEG = 0.019;
BASE_HEIGHT = 70;
L1 = 90;
L2 = 110;
L3 = 110;
z = 70;
for i = 1:length(xy)
    fprintf('i = %d\t', i)
    fprintf('x: %f\t', xy(i,1)*100+100)
    fprintf('y: %f\n', xy(i,2)*100+100)
    x = xy(i,1)*100+100;
    y = xy(i,2)*100+100;
    
    baseAngle = atan2d(y,x);
    baseAngleV = baseAngle*VOLT_PER_DEG + 0.79;
    fprintf('baseAngle = %f\t\t',baseAngle)
    fprintf('baseAngleV = %f\n',baseAngleV)
    radialDist = sqrt(x*x + y*y);
    wristZ = z - BASE_HEIGHT;
    wristY = radialDist - L3;
    swDist = sqrt(wristZ*wristZ + wristY*wristY);
    swAngle1 = atan2d(wristY,wristZ);
    swAngle2 = acosd((L1*L1 + swDist*swDist - L2*L2)/(2*L1*swDist));
    shoulderAngle = 180 - swAngle1 - swAngle2;
    shoulderAngleV = shoulderAngle*VOLT_PER_DEG - 0.79;
    fprintf('shoulderAngle = %f\t',shoulderAngle)
    fprintf('shoulderAngleV = %f\n',shoulderAngleV)
    ewAngle = acosd((L1*L1 + L2*L2 - swDist*swDist)/(2*L1*L2));
    elbowAngle = 90 - ewAngle;
    elbowAngleV = elbowAngle*VOLT_PER_DEG - 0.79;
    fprintf('elbowAngle = %f\t\t',elbowAngle)
    fprintf('elbowAngleV = %f\n',elbowAngleV)
    wristAngle = 90 - shoulderAngle - elbowAngle;
    wristAngleV = wristAngle*VOLT_PER_DEG + 0.79;
    fprintf('wristAngle = %f\t\t',wristAngle)
    fprintf('wristAngleV = %f\n',wristAngleV)
    
end