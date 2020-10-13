%lesson_10_bsplinepolytraj ʹ��b�������ɶ���ʽ�켣
% [q,qd,qdd,pp] = bsplinepolytraj(control points,tInterval,tSamples)����һ���ֶ�����b�����켣��
% �ù켣�����ɿ��Ƶ㶨��Ŀ��ƶ�����С���tInterval�и�������ʼʱ��ͽ���ʱ��֮��Ե������о��Ȳ�����
% ������������ʱ������tSamples��λ�á��ٶȺͼ��ٶȡ��ú��������ض���ʽ�켣����ʱ��ķֶζ���ʽpp��ʽ��

%ʹ��bsplinepolytraj����ָ��һ���άxy���Ƶ㡣b����ʹ����Щ���Ƶ��ڶ�����ڴ����켣����������·���ʱ��㡣

cpts = [1 4 4 3 -2 0; 0 1 2 4 3 1];
tpts = [0 5];

%����b�����켣���ú�������켣λ��(q)���ٶ�(qd)�����ٶ�(qdd)��ʱ��ʸ��(tvec)�Ͷ���ʽϵ��(pp)������ʽ���������ٶ�ʵ��·���㡣

tvec = 0:0.01:5;
[q, qd, qdd, pp] = bsplinepolytraj(cpts,tpts,tvec);

figure
plot(cpts(1,:),cpts(2,:),'xb-')
hold all
plot(q(1,:), q(2,:))
xlabel('X')
ylabel('Y')
hold off

%����b�����켣��ÿ��Ԫ�ص�λ�á���Щ�켣����ʱ���ϲ����������ηֶζ���ʽ��

figure
plot(tvec,q)
hold all
plot([0:length(cpts)-1],cpts,'x')
xlabel('t')
ylabel('Position Value')
legend('X-positions','Y-positions')
hold off






