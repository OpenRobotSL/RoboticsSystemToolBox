%lesson_13_rottraj ���ɷ�����ת����֮��Ĺ켣
%[R,omega,alpha] = rottraj(r0,rF,tInterval,tSamples)����һ���켣���ù켣��r0��rF��������֮�����㣬
%��Щ�����ʱ�����͸�����ʱ��������

%����������Ԫ��·����������֮����롣

q0 = quaternion([0 pi/4 -pi/8],'euler','ZYX','point');
qF = quaternion([3*pi/2 0 -3*pi/4],'euler','ZYX','point');

tvec = 0:0.01:5;

%w������ٶȣ�����Ǽ��ٶ�
[qInterp1,w1,a1] = rottraj(q0,qF,[0 5],tvec);

plot(tvec,compact(qInterp1))
title('Quaternion Interpolation (Uniform Time Scaling)')
xlabel('t')
ylabel('Quaternion Values')
legend('W','X','Y','Z')

%����ת����֮�����켣
r0 = [1 0 0; 0 1 0; 0 0 1];
rF = [0 0 1; 1 0 0; 0 0 0];
tvec = 0:0.1:1;
%���ɹ켣��ʹ��plotTransforms���ƽ��������ת����ת��Ϊ��Ԫ����ָ����ƽ�ơ�ͼ����ʾ������ϵ�������м���ת��
%w������ٶȣ�����Ǽ��ٶ�
[rInterp1,w1,a1] = rottraj(r0,rF,[0 1],tvec);

rotations = rotm2quat(rInterp1);
zeroVect = zeros(length(rotations),1);
translations = [zeroVect,zeroVect,zeroVect];

plotTransforms(translations,rotations)
xlabel('X')
ylabel('Y')
zlabel('Z')