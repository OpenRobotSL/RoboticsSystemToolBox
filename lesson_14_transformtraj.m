%lesson_14_transformtraj ����������ξ���֮��Ĺ켣
%[tforms,vel,acc] = transformtraj(T0,TF,tInterval,tSamples)����һ���켣��
%�ù켣������4��4����α任T0��TF֮����룬���еĵ����ʱ�����͸�����ʱ��������
%�����������λ�ù���ת���������˲�ֵ��ʱ������ʱ��ʸ����
t0 = axang2tform([0 1 1 pi/4])*trvec2tform([0 0 0]);
tF = axang2tform([1 0 1 6*pi/5])*trvec2tform([1 1 1]);
tInterval = [0 1];
tvec = 0:0.01:1;

%�ڵ�֮����롣ʹ��plotTransforms���ƹ켣����ת��ת��Ϊ��Ԫ����ת������ת����ͼ����ʾ������ϵ�������м�任��

[tfInterp, v1, a1] = transformtraj(t0,tF,tInterval,tvec);

rotations = tform2quat(tfInterp);
translations = tform2trvec(tfInterp);

plotTransforms(translations,rotations)
xlabel('X')
ylabel('Y')
zlabel('Z')