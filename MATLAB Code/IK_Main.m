robot = rigidBodyTree("DataFormat","column");
base = robot.Base;

torsoTop = rigidBody("torsoTop");
spineBody = rigidBody("spineBody");

%% All the bodies of the robot:

% The base links to bring the center out to the 4 locations
% herefter reffered to as the "base corners" to distinguish between these
% and the base of the robot
base_1 = rigidBody("base_1");
base_2 = rigidBody("base_2");
base_3 = rigidBody("base_3");
base_4 = rigidBody("base_4");


% The actuator bodies
actuator1_a = rigidBody("actuator1_a");
actuator1_b = rigidBody("actuator1_b");
actuator2_a = rigidBody("actuator2_a");
actuator2_b = rigidBody("actuator2_b");
actuator3_a = rigidBody("actuator3_a");
actuator3_b = rigidBody("actuator3_b");
actuator4_a = rigidBody("actuator4_a");
actuator4_b = rigidBody("actuator4_b");

%The four torso parts
torso1 = rigidBody("torso1");
torso2 = rigidBody("torso2");
torso3 = rigidBody("torso3");
torso4 = rigidBody("torso4");


% collBase = collisionCylinder(0.05,0.04); % cylinder: radius,length
% collBase.Pose = trvec2tform([0 0 0.04/2]);
% coll1 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
% coll1.Pose = trvec2tform([0 0 0.15/2]);
% coll2 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
% coll2.Pose = trvec2tform([0 0 0.15/2]);
% coll3 = collisionBox(0.01,0.02,0.15); % box: length, width, height (x,y,z)
% coll3.Pose = trvec2tform([0 0 0.15/2]);
% collGripper = collisionSphere(0.025); % sphere: radius
% collGripper.Pose = trvec2tform([0 -0.015 0.025/2]);


% addCollision(rotatingBase,collBase)
% addCollision(arm1,coll1)
% addCollision(arm2,coll2)
% addCollision(arm3,coll3)
% addCollision(gripper,collGripper)

%% Creating the joints

%The base joints - Holding the base origin to the 4 base corners
jntFxdBase_1 = rigidBodyJoint("jntFxdBase_1","fixed");
jntFxdBase_2 = rigidBodyJoint("jntFxdBase_2","fixed");
jntFxdBase_3 = rigidBodyJoint("jntFxdBase_3","fixed");
jntFxdBase_4 = rigidBodyJoint("jntFxdBase_4","fixed");

%Actuator base joints
jntAct1Base = rigidBodyJoint("jntAct1Base","revolute");
jntAct2Base = rigidBodyJoint("jntAct2Base","revolute");
jntAct3Base = rigidBodyJoint("jntAct3Base","revolute");
jntAct4Base = rigidBodyJoint("jntAct4Base","revolute");

%Actuator prismatic joints
jntAct1Pris = rigidBodyJoint("jntAct1Pris","prismatic");
jntAct2Pris = rigidBodyJoint("jntAct2Pris","prismatic");
jntAct3Pris = rigidBodyJoint("jntAct3Pris","prismatic");
jntAct4Pris = rigidBodyJoint("jntAct4Pris","prismatic");

%Actuator top joints
jntAct1Top = rigidBodyJoint("jntAct1Top","revolute");
jntAct2Top = rigidBodyJoint("jntAct2Top","revolute");
jntAct3Top = rigidBodyJoint("jntAct3Top","revolute");
jntAct4Top = rigidBodyJoint("jntAct4Top","revolute");

%spine joints
jntSpineBase = rigidBodyJoint("jntSpineBase","revolute");
jntSpineTop = rigidBodyJoint("jntSpineTop","revolute");

%% Setting the movable axes for each joint

%The base joint of the actuators
jntAct1Base.JointAxis = [1 1 1]; % All axes
jntAct2Base.JointAxis = [1 1 1];
jntAct3Base.JointAxis = [1 1 1];
jntAct4Base.JointAxis = [1 1 1]; 

%The prismatic joint of the actuators
jntAct1Pris.JointAxis = [0 0 1]; % Z-axis
jntAct2Pris.JointAxis = [0 0 1];
jntAct3Pris.JointAxis = [0 0 1];
jntAct4Pris.JointAxis = [0 0 1]; 

%The top joint of the actuators
jntAct1Top.JointAxis = [1 1 1]; % All axes
jntAct2Top.JointAxis = [1 1 1];
jntAct3Top.JointAxis = [1 1 1];
jntAct4Top.JointAxis = [1 1 1]; 

%The spine joints
jntSpineBase.JointAxis = [1 1 1];
jntSpineTop.JointAxis = [1 1 1];

%% Setting transforms-----------------------

%Base to Base Corner Transforms
setFixedTransform(jntFxdBase_1,trvec2tform([-0.15 -0.075 0.01]))
setFixedTransform(jntFxdBase_2,trvec2tform([0.15 -0.075 0.01]))
setFixedTransform(jntFxdBase_3,trvec2tform([-0.15 0.075 0.01]))
setFixedTransform(jntFxdBase_4,trvec2tform([0.15 0.075 0.01]))


%Base corner to prismatic joints
setFixedTransform(jntAct1Base,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct2Base,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct3Base,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct4Base,trvec2tform([0 0 0.10]))

%Prismatic joints to top joints
setFixedTransform(jntAct1Pris,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct2Pris,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct3Pris,trvec2tform([0 0 0.10]))
setFixedTransform(jntAct4Pris,trvec2tform([0 0 0.10]))

%top joints to torso plate
setFixedTransform(jntAct1Top,trvec2tform([0.15 0.075 0]))
setFixedTransform(jntAct2Top,trvec2tform([-0.15 0.075 0]))
setFixedTransform(jntAct3Top,trvec2tform([0.15 -0.075 0]))
setFixedTransform(jntAct4Top,trvec2tform([-0.15 -0.075 0]))

% [0.15 0.075 0]
% [-0.15 0.075 0]
% [0.15 0.075 0]
% [-0.15 0.075 0]



% setFixedTransform(jnt2,trvec2tform([-0.015 0 0.15]))
% setFixedTransform(jnt3,trvec2tform([0.015 0 0.15]))
% setFixedTransform(jntGripper,trvec2tform([0 0 0.15]))


%% Assigning the joints to the correct body

%The base to base corner bois
base_1.Joint = jntFxdBase_1;
base_2.Joint = jntFxdBase_2;
base_3.Joint = jntFxdBase_3;
base_4.Joint = jntFxdBase_4;

%The base spherical joints to the lower actuator bodies
actuator1_a.Joint = jntAct1Base;
actuator2_a.Joint = jntAct2Base;
actuator3_a.Joint = jntAct3Base;
actuator4_a.Joint = jntAct4Base;

%The upper actuator part joints (the prismatic bois)
actuator1_b.Joint = jntAct1Pris;
actuator2_b.Joint = jntAct2Pris;
actuator3_b.Joint = jntAct3Pris;
actuator4_b.Joint = jntAct4Pris;

%The upper revolute joints that attach to the top plate
torso1.Joint = jntAct1Top;
torso2.Joint = jntAct2Top;
torso3.Joint = jntAct3Top;
torso4.Joint = jntAct4Top;


%% Adding the bodies to the robot

%Base corners
addBody(robot, base_1, base.Name);
addBody(robot, base_2, base.Name);
addBody(robot, base_3, base.Name);
addBody(robot, base_4, base.Name);

%Lower actuators
addBody(robot, actuator1_a, base_1.Name);
addBody(robot, actuator2_a, base_2.Name);
addBody(robot, actuator3_a, base_3.Name);
addBody(robot, actuator4_a, base_4.Name);

%Top of actuators (connected through the prismatic joints)
addBody(robot, actuator1_b, actuator1_a.Name);
addBody(robot, actuator2_b, actuator2_a.Name);
addBody(robot, actuator3_b, actuator3_a.Name);
addBody(robot, actuator4_b, actuator4_a.Name);

%Top torso bodies going back to center
addBody(robot, torso1, actuator1_b.Name);
addBody(robot, torso2, actuator2_b.Name);
addBody(robot, torso3, actuator3_b.Name);
addBody(robot, torso4, actuator4_b.Name);



% bodies = {base,rotatingBase,spineBody,actuator1_a,actuator2_a,actuator3_a};
% joints = {[],jntAct1Base,jnt1,jnt2,jnt3,jntGripper};
% 
% figure("Name","Assemble Robot","Visible","on")
% for i = 2:length(bodies) % Skip base. Iterate through adding bodies and joints.
%             bodies{i}.Joint = joints{i};
%             addBody(robot,bodies{i},bodies{i-1}.Name)
%             show(robot,"Frames","on");
%             drawnow;
% end

showdetails(robot)
show(robot);


figure("Name","Interactive GUI")
gui = interactiveRigidBodyTree(robot,"MarkerScaleFactor",0.25);


