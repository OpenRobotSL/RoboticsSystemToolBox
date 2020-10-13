%lesson_15_trapveltraj ���ɴ��������ٶ�����Ĺ켣
% [q,qd,qdd,tSamples,pp] = trapveltraj(wayPoints,numSamples)ͨ��������һ����ѭ�����ٶ����������wayPoints���ɹ켣��
% �ú����������ʱ���ڵ�λ�á��ٶȺͼ��ٶȡ��ú��������ض���ʽ�켣����ʱ��ķֶζ���ʽpp��ʽ��

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

%����������֤��άƽ���е�ʵ��λ�á���q������·����ĵ����л���Ϊx��yλ�á�
figure
plot(q(1,:),q(2,:),'-b',wpts(1,:),wpts(2,:),'or')