%% Monte-carlo simulation of the HURON-Upper torso parallel robot
% This simlation generates random transformations fo the torso plate that
% are checked against minimum and maximum lengths of the actuators and
% other constraints. If they are found to be valid movements, they are
% added to an array of transformations from the home top plate location.

% Actuator Information:
% The actuators are numbered as follows:
%
%    4          3
%       center
%    1          2
%
%

validMovements = zeros(4,10000); 
invalidMovements = zeros(4,10000);
validTransformations = zeros(4, 4, 10000);
% This stores the 4 actuator lengths for all the valid movements found

%% Allowed ranges for variables
% all are in meters or degrees unless otherwise stated

% This section will be deleted. The spine doesn't really translate, it just
% rotates relative to the top and bottom. The top and bottom rotations will
% take care of this
xStart = 0;
xMinMax = 0.02;
yStart = 0;
yMinMax = 0.02;
zStart = 0.26;
zMinMax = 0.01;

% Rotation of the top plate relative to the spine
xRStart = 0;
xRMinMax = 5;
yRStart = 0;
yRMinMax = 5;
zRStart = 0;
zRMinMax = 40;% twist

% Rotation of the spine relative to the bottom plate
xRStartLow = 0;
xRMinMaxLow = 5;
yRStartLow = 0;
yRMinMaxLow = 5;
zRStartLow = 0;
zRMinMaxLow = 40;% twist

actuatorMinLength = 0.2385;
actuatorMaxLength = 0.3285;

%% Actuator placement Information:

% Top locations
act1Top = [-0.02 -0.12 0]; % location of the first actuator top relative to top plate
act2Top = [0.02 -0.12 0];
act3Top = [0.1 -0.02 0];
act4Top = [-0.1 -0.02 0];

% bottom locations
act1bottom = [-0.11 -0.1 0]; % location of the first actuator top relative to bottom plate
act2bottom = [0.11 -0.1 0];
act3bottom = [0.12 -0.07 0];
act4bottom = [-0.12 -0.07 0];

Actuator1TMatrixTop = [eye(3) transpose(act1Top); 0 0 0 1];
Actuator2TMatrixTop = [eye(3) transpose(act2Top); 0 0 0 1];
Actuator3TMatrixTop = [eye(3) transpose(act3Top); 0 0 0 1];
Actuator4TMatrixTop = [eye(3) transpose(act4Top); 0 0 0 1];

Actuator1TMatrixBottom = [eye(3) transpose(act1bottom); 0 0 0 1];
Actuator2TMatrixBottom = [eye(3) transpose(act2bottom); 0 0 0 1];
Actuator3TMatrixBottom = [eye(3) transpose(act3bottom); 0 0 0 1];
Actuator4TMatrixBottom = [eye(3) transpose(act4bottom); 0 0 0 1];

%% Random value generation
numberOfRuns = 1000;
tries = 0;
numValid = 1;
numInvalid = 1;
while numValid<numberOfRuns
    xRand = xStart-xMinMax + (xMinMax*2)*rand();
    yRand = yStart-yMinMax + (yMinMax*2)*rand();
    zRand = zStart-zMinMax + (zMinMax*2)*rand();
    
    xRotationRand = xRStart-xRMinMax + (xRMinMax*2)*rand();
    yRotationRand = yRStart-yRMinMax + (yRMinMax*2)*rand();
    zRotationRand = zRStart-zRMinMax + (zRMinMax*2)*rand();
    
    quat = quaternion([xRotationRand,yRotationRand,zRotationRand],'eulerd','XYZ','frame');
    
    rotationMatrix = rotmat(quat,'frame');
    translationMatrix = [xRand yRand zRand]; % column vector representing the translation of the top plane
    transformationMatrix = [rotationMatrix transpose(translationMatrix); 0 0 0 1];
    
    act1TF = transformationMatrix*Actuator1TMatrixTop;
    act2TF = transformationMatrix*Actuator2TMatrixTop;
    act3TF = transformationMatrix*Actuator3TMatrixTop;
    act4TF = transformationMatrix*Actuator4TMatrixTop;
    
    act1 = act1TF(1:3,end)-Actuator1TMatrixBottom(1:3,end);
    act2 = act2TF(1:3,end)-Actuator2TMatrixBottom(1:3,end);
    act3 = act3TF(1:3,end)-Actuator3TMatrixBottom(1:3,end);
    act4 = act4TF(1:3,end)-Actuator4TMatrixBottom(1:3,end);

    act1Len = sqrt(act1(1)^2 + act1(2)^2 + act1(3)^2);
    act2Len = sqrt(act2(1)^2 + act2(2)^2 + act2(3)^2);
    act3Len = sqrt(act3(1)^2 + act3(2)^2 + act3(3)^2);
    act4Len = sqrt(act4(1)^2 + act4(2)^2 + act4(3)^2);
    
    if act1Len>=actuatorMinLength && act2Len>=actuatorMinLength && ...
        act3Len>=actuatorMinLength && act4Len>=actuatorMinLength && ...
        act1Len<=actuatorMaxLength && act2Len<=actuatorMaxLength && ...
        act3Len<=actuatorMaxLength && act4Len<=actuatorMaxLength
        validMovements(:,numValid) = [act1Len; act2Len; act3Len; act4Len];
        validTransformations(:,:,numValid) = transformationMatrix;
        numValid = numValid + 1;
        %disp('valid');
    else
        invalidMovements(:,numInvalid) = [act1Len; act2Len; act3Len; act4Len];
        numInvalid = numInvalid + 1;
        %disp('invalid');
    end


end

%% Initial Plot
f = figure(1);
clf(f)
act1TF = transformationMatrix*Actuator1TMatrixTop;
act2TF = transformationMatrix*Actuator2TMatrixTop;
act3TF = transformationMatrix*Actuator3TMatrixTop;
act4TF = transformationMatrix*Actuator4TMatrixTop;



%plotTransforms([0 0 0], rotm2quat(eye(3)), 'FrameSize',0.05);
hold on
%     plotAxes(Actuator1TMatrixBottom, act1TF);
%     plotAxes(Actuator2TMatrixBottom, act2TF);
%     plotAxes(Actuator3TMatrixBottom, act3TF);
%     plotAxes(Actuator4TMatrixBottom, act4TF);
axis([-0.3 0.3 -0.3 0.3 -0.1 0.4])

% Drawing acuator lines
act1Coords = [transpose(Actuator1TMatrixBottom(1:3,end));
    transpose(act1TF(1:3,end))];
act2Coords = [transpose(Actuator2TMatrixBottom(1:3,end));
    transpose(act2TF(1:3,end))];
act3Coords = [transpose(Actuator3TMatrixBottom(1:3,end));
    transpose(act3TF(1:3,end))];
act4Coords = [transpose(Actuator4TMatrixBottom(1:3,end));
    transpose(act4TF(1:3,end))];
spineCoords = [0 0 0; transpose(transformationMatrix(1:3,end))];
bottomActuatorCoords = [Actuator1TMatrixBottom(1:3,end) Actuator2TMatrixBottom(1:3,end) Actuator3TMatrixBottom(1:3,end) transpose(spineCoords(1,1:3)) Actuator4TMatrixBottom(1:3,end) Actuator1TMatrixBottom(1:3,end)];
topActuatorCoords = [act1TF(1:3,end) act2TF(1:3,end) act3TF(1:3,end) transpose(spineCoords(2,1:3)) act4TF(1:3,end) act1TF(1:3,end)];

draw1 = plot3(act1Coords(:,1),act1Coords(:,2),act1Coords(:,3),'k','LineWidth',3);
draw2 = plot3(act2Coords(:,1),act2Coords(:,2),act2Coords(:,3),'k','LineWidth',3);
draw3 = plot3(act3Coords(:,1),act3Coords(:,2),act3Coords(:,3),'k','LineWidth',3);
draw4 = plot3(act4Coords(:,1),act4Coords(:,2),act4Coords(:,3),'k','LineWidth',3);

draw5 = plot3(topActuatorCoords(1,:), topActuatorCoords(2,:), topActuatorCoords(3,:),'b','LineWidth',3);
draw6 = plot3(bottomActuatorCoords(1,:), bottomActuatorCoords(2,:), bottomActuatorCoords(3,:),'b','LineWidth',3);
draw7 = plot3(spineCoords(:,1),spineCoords(:,2),spineCoords(:,3),'r','LineWidth',10);
hold off
title(0);


%% Plot based on slider value
p = uipanel(f,'Position',[0 0 1 0.05]);
c = uicontrol(p,'Style','slider', 'Position',[0 0 550 20], 'SliderStep',[0.001 0.01]);
c.Value = 1;
c.Min = 1;
c.Max = 999;
c.ButtonDownFcn

oldValue = 1;

while true
    oldValue = c.Value;
    transformationMatrix = validTransformations(:,:,round(oldValue));
    act1TF = transformationMatrix*Actuator1TMatrixTop;
    act2TF = transformationMatrix*Actuator2TMatrixTop;
    act3TF = transformationMatrix*Actuator3TMatrixTop;
    act4TF = transformationMatrix*Actuator4TMatrixTop;

    %plotTransforms([0 0 0], rotm2quat(eye(3)), 'FrameSize',0.05);
    hold on
%     plotAxes(Actuator1TMatrixBottom, act1TF);
%     plotAxes(Actuator2TMatrixBottom, act2TF);
%     plotAxes(Actuator3TMatrixBottom, act3TF);
%     plotAxes(Actuator4TMatrixBottom, act4TF);
    axis([-0.3 0.3 -0.3 0.3 -0.1 0.4])
    
    % Drawing acuator lines
    act1Coords = [transpose(Actuator1TMatrixBottom(1:3,end));
        transpose(act1TF(1:3,end))];
    act2Coords = [transpose(Actuator2TMatrixBottom(1:3,end));
        transpose(act2TF(1:3,end))];
    act3Coords = [transpose(Actuator3TMatrixBottom(1:3,end));
        transpose(act3TF(1:3,end))];
    act4Coords = [transpose(Actuator4TMatrixBottom(1:3,end));
        transpose(act4TF(1:3,end))];
    spineCoords = [0 0 0; transpose(transformationMatrix(1:3,end))];
    bottomActuatorCoords = [Actuator1TMatrixBottom(1:3,end) Actuator2TMatrixBottom(1:3,end) Actuator3TMatrixBottom(1:3,end) transpose(spineCoords(1,1:3)) Actuator4TMatrixBottom(1:3,end) Actuator1TMatrixBottom(1:3,end)];
    topActuatorCoords = [act1TF(1:3,end) act2TF(1:3,end) act3TF(1:3,end) transpose(spineCoords(2,1:3)) act4TF(1:3,end) act1TF(1:3,end)];

    
    
    set(draw1, 'XData', act1Coords(:,1), 'YData', act1Coords(:,2), 'ZData', act1Coords(:,3));
    set(draw2, 'XData', act2Coords(:,1), 'YData', act2Coords(:,2), 'ZData', act2Coords(:,3));
    set(draw3, 'XData', act3Coords(:,1), 'YData', act3Coords(:,2), 'ZData', act3Coords(:,3));
    set(draw4, 'XData', act4Coords(:,1), 'YData', act4Coords(:,2), 'ZData', act4Coords(:,3));
    
    set(draw7, 'XData', spineCoords(:,1), 'YData', spineCoords(:,2), 'ZData', spineCoords(:,3));

    set(draw5, 'XData', topActuatorCoords(1,:), 'YData', topActuatorCoords(2,:), 'ZData', topActuatorCoords(3,:));
    set(draw6, 'XData', bottomActuatorCoords(1,:), 'YData', bottomActuatorCoords(2,:), 'ZData', bottomActuatorCoords(3,:));
    title(strcat(num2str(round(oldValue)),'/ ', num2str(numValid-1)));
    pause(0.01)
end







%% Continuous Plot
% 
% for i=1:numValid-1
% 
%     transformationMatrix = validTransformations(:,:,i);
%     act1TF = transformationMatrix*Actuator1TMatrixTop;
%     act2TF = transformationMatrix*Actuator2TMatrixTop;
%     act3TF = transformationMatrix*Actuator3TMatrixTop;
%     act4TF = transformationMatrix*Actuator4TMatrixTop;
% 
%     %plotTransforms([0 0 0], rotm2quat(eye(3)), 'FrameSize',0.05);
%     hold on
% %     plotAxes(Actuator1TMatrixBottom, act1TF);
% %     plotAxes(Actuator2TMatrixBottom, act2TF);
% %     plotAxes(Actuator3TMatrixBottom, act3TF);
% %     plotAxes(Actuator4TMatrixBottom, act4TF);
%     axis([-0.3 0.3 -0.3 0.3 -0.1 0.4])
%     
%     % Drawing acuator lines
%     act1Coords = [transpose(Actuator1TMatrixBottom(1:3,end));
%         transpose(act1TF(1:3,end))];
%     act2Coords = [transpose(Actuator2TMatrixBottom(1:3,end));
%         transpose(act2TF(1:3,end))];
%     act3Coords = [transpose(Actuator3TMatrixBottom(1:3,end));
%         transpose(act3TF(1:3,end))];
%     act4Coords = [transpose(Actuator4TMatrixBottom(1:3,end));
%         transpose(act4TF(1:3,end))];
%     spineCoords = [0 0 0; transpose(transformationMatrix(1:3,end))];
%     bottomActuatorCoords = [Actuator1TMatrixBottom(1:3,end) Actuator2TMatrixBottom(1:3,end) Actuator3TMatrixBottom(1:3,end) transpose(spineCoords(1,1:3)) Actuator4TMatrixBottom(1:3,end) Actuator1TMatrixBottom(1:3,end)];
%     topActuatorCoords = [act1TF(1:3,end) act2TF(1:3,end) act3TF(1:3,end) transpose(spineCoords(2,1:3)) act4TF(1:3,end) act1TF(1:3,end)];
% 
%     
%     
%     set(draw1, 'XData', act1Coords(:,1), 'YData', act1Coords(:,2), 'ZData', act1Coords(:,3));
%     set(draw2, 'XData', act2Coords(:,1), 'YData', act2Coords(:,2), 'ZData', act2Coords(:,3));
%     set(draw3, 'XData', act3Coords(:,1), 'YData', act3Coords(:,2), 'ZData', act3Coords(:,3));
%     set(draw4, 'XData', act4Coords(:,1), 'YData', act4Coords(:,2), 'ZData', act4Coords(:,3));
%     
%     set(draw7, 'XData', spineCoords(:,1), 'YData', spineCoords(:,2), 'ZData', spineCoords(:,3));
% 
%     set(draw5, 'XData', topActuatorCoords(1,:), 'YData', topActuatorCoords(2,:), 'ZData', topActuatorCoords(3,:));
%     set(draw6, 'XData', bottomActuatorCoords(1,:), 'YData', bottomActuatorCoords(2,:), 'ZData', bottomActuatorCoords(3,:));
%     title(strcat(num2str(i),'/ ', num2str(numValid-1)));
%     pause(0.01)
% 
% end

function plotAxes(t1, tF)
    plotTransforms(transpose(t1(1:3,end)), rotm2quat(t1(1:3,1:3)), 'FrameSize',0.05);
    plotTransforms(transpose(tF(1:3,end)), rotm2quat(tF(1:3,1:3)), 'FrameSize',0.05);
end




