%lesson 4 geometricJacobian
% jacobian = geometricJacobian(robot,configuration,endeffectorname)
% ���������ָ��ĩ��ִ�������ƺͻ�����ģ�����õĻ��ļ����ſɱȾ���

load exampleRobots.mat puma1

%�������⹹����Puma�����ˡ�L6������ļ����ſɱȾ���

geoJacob = geometricJacobian(puma1,randomConfiguration(puma1),'L6')

% ����ָ�����õ�ĩ��ִ�����ļ����ſɱȾ�����6��n�������ʽ���أ�����n��ĩ��ִ���������ɶȡ�
% �ſɱȾ��󽫹ؽڿռ���ٶ�ӳ�䵽ĩ��ִ�������ٶȣ�����ڻ�������ϵ��ĩ��ִ�����ٶȵ���: