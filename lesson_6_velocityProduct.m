%lesson_6_velocityProduct
%抵消速度感应力的关节力矩
%计算在某一关节构型下，为抵消指定关节速度所引起的力所需的关节力矩。重力扭矩不包括在这个计算中。
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
qdot = [0 0 0.2 0.3 0 0.1 0];
%注意是抵消所以是负
tau = -velocityProduct(lbr,[],qdot);