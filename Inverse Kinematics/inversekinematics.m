%% 11-12
function [ configuration ] = inversekinematics( robot, bodyname, initialconfiguration, targetpose, errorweights, redundant, jointweights )
warning('off','all');

q0=JointConf2JointVec(initialconfiguration);

if (nargin<5)
    errorweights=ones(1,6);
end
if (nargin<6)
    redundant=false;
end
if (nargin<7)
    jointweights=ones(1,length(q0));
end

objfun = @(q)poseerror(q,robot,bodyname,targetpose,errorweights,redundant,jointweights);
options = optimoptions('lsqnonlin');
options.Algorithm = 'levenberg-marquardt';
options.Display='off';
options.FunctionTolerance=1e-03;
options.MaxIterations=20;
warning('off','all');
q = lsqnonlin(objfun,q0,[],[],options);
warning('on','all');
configuration=JointVec2JointConf(robot,q);

end

