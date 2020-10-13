%lesson_15_trapveltraj 生成带有梯形速度剖面的轨迹
% [q,qd,qdd,tSamples,pp] = trapveltraj(wayPoints,numSamples)通过给定的一组遵循梯形速度剖面的输入wayPoints生成轨迹。
% 该函数输出给定时间内的位置、速度和加速度。该函数还返回多项式轨迹关于时间的分段多项式pp形式。

wpts = [0 45 15 90 45; 90 45 -45 15 90];
[q, qd, qdd, tvec, pp] = trapveltraj(wpts, 501);

subplot(2,1,1)
plot(tvec, q)
xlabel('t')
ylabel('Positions')
legend('X','Y')
subplot(2,1,2)
plot(tvec, qd)
xlabel('t')
ylabel('Velocities')
legend('X','Y')

%您还可以验证二维平面中的实际位置。将q向量和路径点的单独行绘制为x和y位置。
figure
plot(q(1,:),q(2,:),'-b',wpts(1,:),wpts(2,:),'or')