%% 10
function [ perror ] = poseerror( q, robot, bodyname, targetpose, errorweights, redundant, jointweights )
% q : n-by-1 joint vector 
% robot : rigid body tree object
% bodyname : name of end effector link
% tform : transform of target pose 
% errorweights : n-by-n positive definite weight matrix
% redundant : flag if robot is redundant
% perror : vector of task space errors

configuration=JointVec2JointConf(robot, q);

if (nargin<5)
    errorweights=ones(1,6);
end
if (nargin<6)
    redundant=false;
end
if (nargin<7)
    jointweights=ones(1,length(q));
end

% forward kinematics, transform of end effector frame
T = robot.getTransform(configuration, bodyname);
% pose error
e = robotics.manip.internal.IKHelpers.poseError(targetpose, T);

%% 15
if redundant
    % geometric Jacobian
    J = robot.geometricJacobian(configuration, bodyname);
    % qnull=(eye(length(q))-J'*inv(J*J')*J)*q'
    qnull=q'-J'*(J'\q');
    perror=[errorweights.*e' jointweights.*qnull'];
else
    perror=[errorweights.*e'];
end
end

