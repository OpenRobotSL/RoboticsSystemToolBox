%lesson_5_gravityTorque
%���������Ĺؽ�����
load exampleRobots.mat lbr
lbr.DataFormat = 'row'; 
lbr.Gravity = [0 0 -9.81];
q = randomConfiguration(lbr);
gtau = gravityTorque(lbr,q);