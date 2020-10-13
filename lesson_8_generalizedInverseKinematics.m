%lesson_8_generalizedInverseKinematics ������Լ�����˶�ѧ�����
% �������˶�ѧϵͳ����ʹ��һ���˶�ѧԼ���������ɸ������ָ���ĸ�����ģ�͵Ĺؽڹ��͡�
% �������˶�ѧ������÷����Խ�����������Լ��������ﵽ��ѱƽ���
% �ڵ��ö���֮ǰָ��Լ������ConstraintInputs��Ҫ�ڵ��ö�������Լ�����룬�����release(gik)��
% ��Լ������ָ��ΪԼ�����󣬲����ô�����Щ�����ͨ�����˶�ѧ��Ҫ����Լ��������ʹ�����¶���

% ���һ��Լ���Ĺ������˶�ѧ
% ����һ���������˶�ѧ����������������ֱ۹̶����ض�λ�ò�ָ������˻���������Լ�����󣬽���Ҫ��Լ���������ݵ��������

load exampleRobots.mat lbr
gik = generalizedInverseKinematics;
gik.RigidBodyTree = lbr;
% ��������Լ������
gik.ConstraintInputs = {'position','aiming'};
% ��Ϊtool0�������ԭ������ڻ����˵�baseλ��[0.0 0.5 0.5]��
posTgt = constraintPositionTarget('tool0');
posTgt.TargetPosition = [0.0 0.5 0.5];
% ��Ϊtool0�ĸ���z��ָ������˻�����ܵ�ԭ�㡣Ҳ����ĩ��tool0��Z��ָ��000�����
aimCon = constraintAiming('tool0');
aimCon.TargetPoint = [0.0 0.0 0.8];
%�ҵ�һ������Լ�������á������밴����ConstraintInputs������ָ����˳��Լ�����󴫵ݵ�ϵͳ����ָ�����������õĳ�ʼ�²⡣
q0 = homeConfiguration(lbr); % Initial guess for solver
[q,solutionInfo] = gik(q0,posTgt,aimCon);
% solutionInfo���������Ϣ����Ϊ�ṹ���ء����������Ϣ�ṹ���������ֶ�:
% ���������㷨���еĵ���������
% NumRandomRestarts�������㷨����ֲ���Сֵ������������������Ĵ�����
% PoseErrorNorm���������������ĩ��ִ������̬��ȵ�λ�����Ĵ�С��
% ExitFlag -���룬�ṩ��������㷨ִ�е�ϸ�ڣ��Լ�ʲô���������ء��й�ÿ���㷨���͵��˳���־����μ��˳���־��
% ״̬-�ַ��������������������������Χ��(���ɹ���)�����㷨�ܹ��ҵ�����ѽ������(����ѿ��á�)��
% ���ӻ���������ص����á�
show(lbr,q);
title(['Solver status: ' solutionInfo.Status])
axis([-0.75 0.75 -0.75 0.75 -0.5 1])
% ��Ŀ��λ�õ�����ԭ�����һ���߶Ρ�tool0֡��ԭ�����߶ε�һ���غϣ���z�����߶ζ��롣
hold on
plot3([0.0 0.0],[0.5 0.0],[0.5 0.8],'--o')
hold off