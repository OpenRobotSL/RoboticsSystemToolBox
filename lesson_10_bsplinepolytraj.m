%lesson_10_bsplinepolytraj 使用b样条生成多项式轨迹
% [q,qd,qdd,pp] = bsplinepolytraj(control points,tInterval,tSamples)生成一个分段三次b样条轨迹，
% 该轨迹落在由控制点定义的控制多边形中。在tInterval中给定的起始时间和结束时间之间对弹道进行均匀采样。
% 函数返回输入时间样本tSamples的位置、速度和加速度。该函数还返回多项式轨迹关于时间的分段多项式pp形式。

%使用bsplinepolytraj函数指定一组二维xy控制点。b样条使用这些控制点在多边形内创建轨迹。还给出了路点的时间点。

cpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
tpts = [0 5];

%计算b样条轨迹。该函数输出轨迹位置(q)、速度(qd)、加速度(qdd)、时间矢量(tvec)和多项式系数(pp)，多项式利用梯形速度实现路径点。

tvec = 0:0.01:5;
[q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvec);

figure
plot(cpts(1,:),cpts(2,:),'xb-')
hold all
plot(q(1,:), q(2,:))
xlabel('X')
ylabel('Y')
hold off

%绘制b样条轨迹中每个元素的位置。这些轨迹是在时间上参数化的三次分段多项式。

figure
plot(tvec,q)
hold all
plot([0:length(cpts)-1],cpts,'x')
xlabel('t')
ylabel('Position Value')
legend('X-positions','Y-positions')
hold off






