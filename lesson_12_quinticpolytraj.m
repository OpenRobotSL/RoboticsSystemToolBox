%lesson_12_quinticpolytraj ������׶���ʽ�켣
% quinticpolytraj(wayPoints,timePoints,tSamples)����һ����׶���ʽ���ö���ʽʵ��һ�����������wayPoints����Ӧ��ʱ��㡣
% �ú����������ʱ������tSamples��λ�á��ٶȺͼ��ٶȡ��ú��������ض���ʽ�켣����ʱ��ķֶζ���ʽpp��ʽ��

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