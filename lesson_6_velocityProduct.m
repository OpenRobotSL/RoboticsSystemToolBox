%lesson_6_velocityProduct
%�����ٶȸ�Ӧ���Ĺؽ�����
%������ĳһ�ؽڹ����£�Ϊ����ָ���ؽ��ٶ��������������Ĺؽ����ء�����Ť�ز���������������С�
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
qdot = [0 0 0.2 0.3 0 0.1 0];
%ע���ǵ��������Ǹ�
tau = -velocityProduct(lbr,[],qdot);