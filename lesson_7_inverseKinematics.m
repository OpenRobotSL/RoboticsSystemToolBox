%lesson_7_inverseKinematics
load exampleRobots.mat
showdetails(puma1)

randConfig = puma1.randomConfiguration;
tform = getTransform(puma1,randConfig,'L6','base');

show(puma1,randConfig);

ik = inverseKinematics('RigidBodyTree',puma1);
% 位姿误差的权重，指定为一个六元向量。前三个元素对应于所需姿态的方向误差的权重。
% 最后三个元素对应于所需姿态的xyz位置上的错误权重。
weights = [0.25 0.25 0.25 1 1 1];
initialguess = puma1.homeConfiguration;

[configSoln,solnInfo] = ik('L6',tform,weights,initialguess);

show(puma1,configSoln);