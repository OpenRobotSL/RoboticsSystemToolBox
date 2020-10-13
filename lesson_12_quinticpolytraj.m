%lesson_12_quinticpolytraj 生成五阶多项式轨迹
% quinticpolytraj(wayPoints,timePoints,tSamples)生成一个五阶多项式，该多项式实现一组给定的输入wayPoints和相应的时间点。
% 该函数输出给定时间样本tSamples的位置、速度和加速度。该函数还返回多项式轨迹关于时间的分段多项式pp形式。

wpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
tpts = 0:5;
tvec = 0:0.01:5;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tvec,'VelocityBoundaryCondition',[1 0 -1 -1 0 0; 1 1 1 -1 -1 -1]);
plot(tvec, q)
hold all
plot(tpts, wpts, 'x')
xlabel('t')
ylabel('Positions')
legend('X-positions','Y-positions')
hold off

figure
plot(q(1,:),q(2,:),'.b',wpts(1,:),wpts(2,:),'or')
xlabel('X')
ylabel('Y')