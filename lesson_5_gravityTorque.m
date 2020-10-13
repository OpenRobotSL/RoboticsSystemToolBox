%lesson_5_gravityTorque
%补偿重力的关节力矩
load exampleRobots.mat lbr
lbr.DataFormat = 'row'; 
lbr.Gravity = [0 0 -9.81];
q = randomConfiguration(lbr);
gtau = gravityTorque(lbr,q);