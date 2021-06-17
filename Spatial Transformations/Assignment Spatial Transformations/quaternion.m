%Program2
theta1 = pi/4;
a{1,1} = 0;        a{2,1} = -0.612;  a{3,1} = 0.5732;
a{4,1} = 0;        a{5,1} = 0;       a{6,1} = 0;

d{1,1} = 0.1273;   d{2,1} = 0;       d{3,1} = 0;
d{4,1} = 0.163914; d{5,1} = 0.1157;  d{6,1} = 0.0922;


th{1,1} = 0;    th{2,1} = 0;         th{3,1} = 0;
th{4,1} = 0;    th{5,1} = 0;         th{6,1} = 0;


alp{1,1} = pi/2;   alp{2,1} = 0;      alp{3,1} = 0;
alp{4,1} = pi/2;   alp{5,1} = -pi/2;  alp{6,1} = 0;


trvecz = [0 0 d{1,1}];
homotrvecz = trvec2tform(trvecz);
rotmz = axang2rotm([0 0 1 theta1]);
homorotmz = rotm2tform(rotmz);
trvecx = [alp{1,1} 0 0];
homotrvecx = trvec2tform(trvecx);
rotmx = axang2rotm([1 0 0 alp{1,1}]);
homorotmx = rotm2tform(rotmx);
shoulderlink = homotrvecz * homorotmz * homotrvecx * homorotmx;

%Program3
robot = robotics.RigidBodyTree;
baselink = robotics.RigidBody('BaseLink');
worldjoint = robotics.Joint('WorldJoint');
worldjoint.setFixedTransform([0 0 1 pi],'dh');
baselink.Joint = worldjoint;
basename = robot.BaseName;
robot.addBody(baselink,basename);

%Program4
shoulderlink = robotics.RigidBody('ShoulderLink');
shoulderpanjoint = robotics.Joint('ShoulderPan','revolute');
shoulderpanjoint.setFixedTransform([a{1,1} alp{1,1} d{1,1} th{1,1}],'dh');
shoulderlink.Joint = shoulderpanjoint;
robot.addBody(shoulderlink,'BaseLink');

%Program5
robot.showdetails();

%Program6
upperarmlink = robotics.RigidBody('UpperArmLink');
shoulderliftjoint = robotics.Joint('ShoulderLift','revolute');
shoulderpanjoint.setFixedTransform([a{2,1} alp{2,1} d{2,1} th{2,1}],'dh');
upperarmlink.Joint = shoulderliftjoint;
robot.addBody(upperarmlink,'ShoulderLink');

forearmlink = robotics.RigidBody('ForeArmLink');
elbowjoint = robotics.Joint('Elbow','revolute');
elbowjoint.setFixedTransform([a{3,1} alp{3,1} d{3,1} th{3,1}],'dh');
forearmlink.Joint = elbowjoint;
robot.addBody(forearmlink,'UpperArmLink');

wrist1link = robotics.RigidBody('Wrist1Link');
wrist1joint = robotics.Joint('Wrist1','revolute');
wrist1joint.setFixedTransform([a{4,1} alp{4,1} d{4,1} th{4,1}],'dh');
wrist1link.Joint = wrist1joint;
robot.addBody(wrist1link,'ForeArmLink');


wrist2link = robotics.RigidBody('Wrist2Link');
wrist2joint = robotics.Joint('Wrist2','revolute');
wrist2joint.setFixedTransform([a{5,1} alp{5,1} d{5,1} th{5,1}],'dh');
wrist2link.Joint = wrist2joint;
robot.addBody(wrist2link,'Wrist1Link');


wrist3link = robotics.RigidBody('Wrist3Link');
wrist3joint = robotics.Joint('Wrist3','revolute');
wrist3joint.setFixedTransform([a{6,1} alp{6,1} d{6,1} th{6,1}],'dh');
wrist3link.Joint = wrist3joint;
robot.addBody(wrist3link,'Wrist2Link');

%Program7

eelink = robotics.RigidBody('EndEffectorLink');
eejoint = robotics.Joint('EndEffector','revolute');
eejoint.setFixedTransform([0 0 0 0],'dh');
eelink.Joint = eejoint;
robot.addBody(eelink,'Wrist3Link');

%Program8
robot.showdetails();