% Assignment Inverse Kinematics

%% 1)
ur10=importrobot('ur10.urdf');
ur10.showdetails();

%% 2)
ikur10 = robotics.InverseKinematics('RigidBodyTree',ur10);

%% 3)
t1=trvec2tform([0.8 0 1.4]);
t2=eul2tform([pi/4, pi/4, -pi/4]);
targetposeur10=t1*t2;

%% 4)
weights = ones(6,1);
initialposeur10 = ur10.homeConfiguration;
[configur10, solnInfo] = ikur10('ee_link',targetposeur10,weights,initialposeur10);

figure(1);
hold on;
ur10.show(configur10);
hold off;

%% 5)
isposeur10 = ur10.getTransform(configur10,'ee_link');
poseerror = robotics.manip.internal.IKHelpers.poseError(targetposeur10, isposeur10)';
[ikconfigur10, solnInfo] = ikur10('ee_link',isposeur10,weights,initialposeur10);
jointerror=JointConf2JointVec(configur10)-JointConf2JointVec(ikconfigur10);
disp('UR10 Robot');
disp(' ');
disp(['Robotics System Toolbox ik norm(e) :  ', num2str(norm(poseerror))]);
disp(['Robotics System Toolbox ik norm(q) :  ', num2str(norm(jointerror))]);
disp(' ');


%% 10-13) alternative with optim toolbox
ur10.show(configur10);
configur10 = inversekinematics(ur10, 'ee_link',initialposeur10,targetposeur10);
isposeur10 = ur10.getTransform(configur10,'ee_link');
poseerror = robotics.manip.internal.IKHelpers.poseError(targetposeur10, isposeur10)';
ikconfigur10 = inversekinematics(ur10, 'ee_link',initialposeur10,isposeur10);
jointerror=JointConf2JointVec(configur10)-JointConf2JointVec(ikconfigur10);
disp(['LM implementation non-redundant norm(e) :  ', num2str(norm(poseerror))]);
disp(['LM implementation non-redundant norm(q) :  ', num2str(norm(jointerror))]);
disp(' ');



%% 6)
sawyer=importrobot('sawyer.urdf');
% showdetails(sawyer);

%% 7)
% delete headpan joint
sawyer.removeBody('head');
sawyer.removeBody('right_arm_itb');
sawyer.removeBody('right_hand_camera');
sawyer.removeBody('torso');
sawyer.removeBody('right_torso_itb');
sawyer.show;

%% 8)
iksawyer = robotics.InverseKinematics('RigidBodyTree',sawyer);
initialposesawyer = sawyer.homeConfiguration;
% initialposesaywer(1).JointPosition=pi;
% initialposesaywer(2).JointPosition=pi;
% initialposesaywer(3).JointPosition=-pi;

t1=trvec2tform([0.6 0.0 0.6]);
t2=eul2tform([pi/4, pi/6, -pi/4]);
targetposesawyer=t1*t2;
[configsawyer, solnInfo] = iksawyer('right_hand',targetposesawyer,weights,initialposesawyer);

%% 9)
isposesawyer10 = sawyer.getTransform(configsawyer,'right_hand');
poseerror=robotics.manip.internal.IKHelpers.poseError(targetposesawyer, isposesawyer10)';
[ikconfigsawyer10, solnInfo] = iksawyer('right_hand',isposesawyer10,weights,initialposesawyer);
jointerror=JointConf2JointVec(configsawyer)-JointConf2JointVec(ikconfigsawyer10);
disp('Saywer Robot');
disp(' ');
disp(['Robotics System Toolbox q :  ', num2str(JointConf2JointVec(configsawyer))]);
disp(['Robotics System Toolbox ik norm(e) :  ', num2str(norm(poseerror))]);
disp(['Robotics System Toolbox ik  norm(q) :  ', num2str(norm(jointerror))]);
disp(['Robotics System Toolbox ik  norm(q(1:3)) :  ', num2str(norm(jointerror(1:3)))]);
disp(' ');


%% 18a
figure(2);
sawyer.show(configsawyer);



%% 14) alternative with optim toolbox
configsawyer=inversekinematics(sawyer,'right_hand',initialposesawyer,targetposesawyer,ones(1,6));
isposesawyer10 = sawyer.getTransform(configsawyer,'right_hand');
poseerror=robotics.manip.internal.IKHelpers.poseError(targetposesawyer, isposesawyer10)';
ikconfigsawyer10 = inversekinematics(sawyer,'right_hand',initialposesawyer,isposesawyer10,ones(1,6));
jointerror=JointConf2JointVec(configsawyer)-JointConf2JointVec(ikconfigsawyer10);
disp(['LM implementation non-redundant q :  ', num2str(JointConf2JointVec(configsawyer))]);
disp(['LM implementation non-redundant norm(e) :  ', num2str(norm(poseerror))]);
disp(['LM implementation non-redundant norm(q) :  ', num2str(norm(jointerror))]);
disp(['LM implementation non-redundant norm(q(1:3)) :  ', num2str(norm(jointerror(1:3)))]);
disp(' ');

%% 15) - 17)
configsawyer=inversekinematics(sawyer,'right_hand',initialposesawyer,targetposesawyer,ones(1,6),true,[1 1 1 0 0 0 0]);
isposesawyer10 = sawyer.getTransform(configsawyer,'right_hand');
poseerror=robotics.manip.internal.IKHelpers.poseError(targetposesawyer, isposesawyer10)';
ikconfigsawyer10 = inversekinematics(sawyer,'right_hand',initialposesawyer,isposesawyer10,ones(1,6),true,[1 1 1 0 0 0 0]);
jointerror=JointConf2JointVec(configsawyer)-JointConf2JointVec(ikconfigsawyer10);
disp(['LM implementation redundant q :  ', num2str(JointConf2JointVec(configsawyer))]);
disp(['LM implementation redundant norm(e) :  ', num2str(norm(poseerror))]);
disp(['LM implementation redundant norm(q) :  ', num2str(norm(jointerror))]);
disp(['LM implementation redundant norm(q(1:3)) :  ', num2str(norm(jointerror(1:3)))]);

%% 18b
hold on;
show(sawyer,configsawyer);
hold off;
