function [ configuration ] = JointVec2JointConf( robot, q )
% robot : RigidBodyTree
% q : joint vector
% configuration : configuration structue
% create default configuration struct
configuration=homeConfiguration(robot);
assert(isequal(length(configuration),length(q)),'mismatch of length of robot joint variables and joint vector');
for i=1:length(configuration)
    configuration(i).JointPosition=q(i);
end

end

