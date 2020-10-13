%lesson_13_rottraj 生成方向旋转矩阵之间的轨迹
%[R,omega,alpha] = rottraj(r0,rF,tInterval,tSamples)生成一个轨迹，该轨迹在r0和rF两个方向之间插入点，
%这些点基于时间间隔和给定的时间样本。

%定义两个四元数路径点在它们之间插入。

q0 = quaternion([0 pi/4 -pi/8],'euler','ZYX','point');
qF = quaternion([3*pi/2 0 -3*pi/4],'euler','ZYX','point');

tvec = 0:0.01:5;

%w方向角速度，方向角加速度
[qInterp1,w1,a1] = rottraj(q0,qF,[0 5],tvec);

plot(tvec,compact(qInterp1))
title('Quaternion Interpolation (Uniform Time Scaling)')
xlabel('t')
ylabel('Quaternion Values')
legend('W','X','Y','Z')

%在旋转矩阵之间插入轨迹
r0 = [1 0 0; 0 1 0; 0 0 1];
rF = [0 0 1; 1 0 0; 0 0 0];
tvec = 0:0.1:1;
%生成轨迹。使用plotTransforms绘制结果。将旋转矩阵转换为四元数并指定零平移。图中显示了坐标系的所有中间旋转。
%w方向角速度，方向角加速度
[rInterp1,w1,a1] = rottraj(r0,rF,[0 1],tvec);

rotations = rotm2quat(rInterp1);
zeroVect = zeros(length(rotations),1);
translations = [zeroVect,zeroVect,zeroVect];

plotTransforms(translations,rotations)
xlabel('X')
ylabel('Y')
zlabel('Z')