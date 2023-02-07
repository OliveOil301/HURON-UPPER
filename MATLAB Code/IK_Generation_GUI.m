%% This script will be used to send movements to the HURON Upper Body torso
% This script allows the user to do the following:
% make a movement of the torso using a GUI
% see if that movement is valid
% send the movement information to the esp32 controlling the robot



%% Variable definitions

%minimum and maximum actuator lengths to check if a given transformation is
%possible for the actuators to reach
actuatorMinLength = 0.2385;
actuatorMaxLength = 0.3285;


% Minimum and maximum rotation amounts for roll, pitch, yaw
minRoll = -20;
maxRoll = 20;
minPitch = -20;
maxPitch = 20;
minYaw = -20;
maxYaw = 20;

% Locations of the top of the actuators relative to where the spine meets
% the top plate
act1Top = [-0.02 -0.12 0]; % location of the first actuator top relative to top plate
act2Top = [0.02 -0.12 0];
act3Top = [0.1 -0.02 0];
act4Top = [-0.1 -0.02 0];

% Locations of the bottom of the actuators relative to where the spine meets
% the bottom plate
act1bottom = [-0.11 -0.1 0]; % location of the first actuator top relative to bottom plate
act2bottom = [0.11 -0.1 0];
act3bottom = [0.12 -0.07 0];
act4bottom = [-0.12 -0.07 0];

% Transformation matricies for the previous translations
Actuator1TMatrixTop = [eye(3) transpose(act1Top); 0 0 0 1];
Actuator2TMatrixTop = [eye(3) transpose(act2Top); 0 0 0 1];
Actuator3TMatrixTop = [eye(3) transpose(act3Top); 0 0 0 1];
Actuator4TMatrixTop = [eye(3) transpose(act4Top); 0 0 0 1];

% Transformation matricies for the previous translations
Actuator1TMatrixBottom = [eye(3) transpose(act1bottom); 0 0 0 1];
Actuator2TMatrixBottom = [eye(3) transpose(act2bottom); 0 0 0 1];
Actuator3TMatrixBottom = [eye(3) transpose(act3bottom); 0 0 0 1];
Actuator4TMatrixBottom = [eye(3) transpose(act4bottom); 0 0 0 1];


%% Initial Setup

% making all the transformation matricies for the first time and drawing
% everything on the figure. 

% The initial transformation of the spine where it is just straight
% vertical
transformationMatrix = [1 0 0 0; 0 1 0 0; 0 0 1 0.26; 0 0 0 1];

%The transformation from the bottom of the spine (origin) to the top of each
%actuators
act1TF = transformationMatrix*Actuator1TMatrixTop;
act2TF = transformationMatrix*Actuator2TMatrixTop;
act3TF = transformationMatrix*Actuator3TMatrixTop;
act4TF = transformationMatrix*Actuator4TMatrixTop;

%Make the figure and clear it, just in case the old figure wasn't closed
f = figure(1);
f.Position = [100 100 800 500];
clf(f)
g = uipanel(f,'Units','normalized','Position',[0 0 0.7,1]); % graphing panel
p = uipanel(f,'Units','normalized','Position',[0.7 0 0.3,1]); % control panel

hold on
ax = axes(g); %This makes sure the graphing happens in the 'g' uipanel obj
axis([-0.3 0.3 -0.3 0.3 -0.1 0.4]); % Setting axis limits

%getting the coordinates of all the actuators and the spine
act1Coords = [transpose(Actuator1TMatrixBottom(1:3,end));
    transpose(act1TF(1:3,end))];
act2Coords = [transpose(Actuator2TMatrixBottom(1:3,end));
    transpose(act2TF(1:3,end))];
act3Coords = [transpose(Actuator3TMatrixBottom(1:3,end));
    transpose(act3TF(1:3,end))];
act4Coords = [transpose(Actuator4TMatrixBottom(1:3,end));
    transpose(act4TF(1:3,end))];
spineCoords = [0 0 0; transpose(transformationMatrix(1:3,end))];
% The coordinates for the top of all 4 actuators
bottomActuatorCoords = [Actuator1TMatrixBottom(1:3,end) Actuator2TMatrixBottom(1:3,end) Actuator3TMatrixBottom(1:3,end) transpose(spineCoords(1,1:3)) Actuator4TMatrixBottom(1:3,end) Actuator1TMatrixBottom(1:3,end)];
% The coordinates for the bottom of all 4 actuators
topActuatorCoords = [act1TF(1:3,end) act2TF(1:3,end) act3TF(1:3,end) transpose(spineCoords(2,1:3)) act4TF(1:3,end) act1TF(1:3,end)];

%drawing all 4 actuators using the coordinates found above
draw1 = plot3(act1Coords(:,1),act1Coords(:,2),act1Coords(:,3),'k','LineWidth',3);
draw2 = plot3(act2Coords(:,1),act2Coords(:,2),act2Coords(:,3),'k','LineWidth',3);
draw3 = plot3(act3Coords(:,1),act3Coords(:,2),act3Coords(:,3),'k','LineWidth',3);
draw4 = plot3(act4Coords(:,1),act4Coords(:,2),act4Coords(:,3),'k','LineWidth',3);

% Drawing the top plate in blue
draw5 = plot3(topActuatorCoords(1,:), topActuatorCoords(2,:), topActuatorCoords(3,:),'b','LineWidth',3);
% Drawing the bottom plate in blue
draw6 = plot3(bottomActuatorCoords(1,:), bottomActuatorCoords(2,:), bottomActuatorCoords(3,:),'b','LineWidth',3);
% Drawing the spine
draw7 = plot3(spineCoords(:,1),spineCoords(:,2),spineCoords(:,3),'r','LineWidth',10);

hold off
title(0);

% Setting up the GUI

% roll (rotation about x-axis) slider & Text
rollSlider = uicontrol(p,'Style','slider','Units','pixels', 'Position',[0 200 237 20], 'SliderStep',[0.01 0.1]);
rollText = uicontrol(p,'Style','text','Units','pixels', 'Position',[0 222 200 15],'String','Roll value in degrees: ','HorizontalAlignment','left');
rollValue = uicontrol(p,'Style','text','Units','pixels', 'Position',[105 222 30 15],'String','0','HorizontalAlignment','left');
rollSlider.Value = 0;
rollSlider.Min = minRoll;
rollSlider.Max = maxRoll;
% pitch (rotation about y-axis) slider
pitchSlider = uicontrol(p,'Style','slider','Units','pixels', 'Position',[0 160 237 20], 'SliderStep',[0.01 0.1]);
pitchText = uicontrol(p,'Style','text','Units','pixels', 'Position',[0 182 200 15],'String','Pitch value in degrees: ','HorizontalAlignment','left');
pitchSlider.Value = 0;
pitchSlider.Min = minPitch;
pitchSlider.Max = maxPitch;
% yaw (rotation about z-axis) slider
yawSlider = uicontrol(p,'Style','slider','Units','normalized', 'Position',[0 120 237 20], 'SliderStep',[0.01 0.1]);
yawText = uicontrol(p,'Style','text','Units','normalized', 'Position',[0 0.25 1 0.03],'String','Yaw value in degrees: ','HorizontalAlignment','left');
yawValue = uicontrol(p,'Style','text','Units','pixels', 'Position',[105 222 30 15],'String','0','HorizontalAlignment','left');
yawSlider.Value = 0;
yawSlider.Min = minYaw;
yawSlider.Max = maxYaw;



%% GUI
while true
    hold on
    %Updating the roll, pitch, yaw text based on the slider value
    rollValue.String = num2str(rollSlider.Value,'%.2f');
    


    pause(0.01)




end