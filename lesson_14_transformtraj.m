%lesson_14_transformtraj 生成两个齐次矩阵之间的轨迹
%[tforms,vel,acc] = transformtraj(T0,TF,tInterval,tSamples)生成一个轨迹，
%该轨迹在两个4×4的齐次变换T0和TF之间插入，其中的点基于时间间隔和给定的时间样本。
%从两个方向和位置构建转换。给出了插值的时间间隔和时间矢量。
t0 = axang2tform([0 1 1 pi/4])*trvec2tform([0 0 0]);
tF = axang2tform([1 0 1 6*pi/5])*trvec2tform([1 1 1]);
tInterval = [0 1];
tvec = 0:0.01:1;

%在点之间插入。使用plotTransforms绘制轨迹。将转换转换为四元数旋转和线性转换。图中显示了坐标系的所有中间变换。

[tfInterp, v1, a1] = transformtraj(t0,tF,tInterval,tvec);

rotations = tform2quat(tfInterp);
translations = tform2trvec(tfInterp);

plotTransforms(translations,rotations)
xlabel('X')
ylabel('Y')
zlabel('Z')