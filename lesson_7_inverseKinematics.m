%lesson_7_inverseKinematics
load exampleRobots.mat
showdetails(puma1)

randConfig = puma1.randomConfiguration;
tform = getTransform(puma1,randConfig,'L6','base');

show(puma1,randConfig);

ik = inverseKinematics('RigidBodyTree',puma1);
% λ������Ȩ�أ�ָ��Ϊһ����Ԫ������ǰ����Ԫ�ض�Ӧ��������̬�ķ�������Ȩ�ء�
% �������Ԫ�ض�Ӧ��������̬��xyzλ���ϵĴ���Ȩ�ء�
weights = [0.25 0.25 0.25 1 1 1];
initialguess = puma1.homeConfiguration;

[configSoln,solnInfo] = ik('L6',tform,weights,initialguess);

show(puma1,configSoln);