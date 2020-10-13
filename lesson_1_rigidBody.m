% lesson 1 rigidBody
% �������κ����ͻ�е�۵Ļ���������ÿ�������϶���һ������ؽڶ��󣬸ö������˸�����˶���ʽ��
% ���ø�����������װ���һ�����ͽṹ�Ļ�����ģ�͡��ڵ���addBody��������ӵ�������ģ��֮ǰ����joint��������Ϊjoint���ԡ�
% �������ڸ������У��㲻��ֱ���޸��������ԣ���Ϊ���ƻ�������֮��Ĺ�ϵ��ʹ��replaceJoint���޸��������ṹ��
% ��������ָ�����Ƶĸ��塣Ĭ������£�������һ���̶��Ĺؽڡ�

% Examples ������ͽ�ͷ���ӵ���������

clear

rbtree = rigidBodyTree

% ����һ������Ψһ���Ƶĸ��塣Ĭ�Ϲؽ������ǹ̶�
body1 = rigidBody('b1')
% ������洴���õĸ��壬���ø���Ĺؽ����Ժ����ƣ�rigidBodyJoint��������rigidBody�е�Joint����
jnt1 = rigidBodyJoint('jnt1','revolute')
body1.Joint = jnt1
% ��������ӵ����С�ָ��Ҫ�����帽�ӵ����������ơ���Ϊ���ǵ�һ�����壬����ʹ�����Ļ�����
basename = rbtree.BaseName%���Ȼ�ȡ����base�����ƣ�Ȼ��Ѵ�����rigid���뵽���У�base�Ǹ�rigid
addBody(rbtree,body1,basename)
% ʹ������showdetails��ȷ�ϸ���͹ؽ��Ƿ���ȷ���
showdetails(rbtree)

% ����Denavit-Hartenberg����������е�ֻ�����
% ʹ��Puma560?�����˵�Denavit-Hartenberg (DH)�������������ˡ�ÿ������һ�����һ�����ɹؽڶ���ָ���ӵ�����ת����
% DH���������˻����˵ļ�����״����ÿ��������θ��ŵ��丸���йء�Ϊ�˷���������ھ���������Puma560�����˵Ĳ�����
% Puma��������һ��������ʽ��е�֡�DH��������ھ����е�ǰһ�У���Ӧ��ǰһ���ؽ����ӡ�

clear
%           a       alpha   d      theta    
dhparams = [0   	pi/2	0   	0;
            0.4318	0       0       0
            0.0203	-pi/2	0.15005	0;
            0   	pi/2	0.4318	0;
            0       -pi/2	0   	0;
            0       0       0       0];

robot = rigidBodyTree;
% ����һ��rigidBody���󲢸���һ��Ψһ�����ơ�
body1 = rigidBody('body1'); %��һ������
jnt1 = rigidBodyJoint('jnt1','revolute');%��������Ĺؽ���ת���ؽ����ֽ�jnt1
% ʹ��setFixedTransformָ��ʹ��DH�����Ĵ����嵽�����ת����DH���������һ��Ԫ�أ��������ˣ���Ϊ�Ƕ������ڹؽڵ�λ�á�
setFixedTransform(jnt1,dhparams(1,:),'dh');%����DH�任���Ӹ����Լ��������������dh��mdh
body1.Joint = jnt1;%�ѹؽ����Դ�������
% ����addBody����һ������ؽ����ӵ������˵Ļ�������ϡ�
addBody(robot,body1,'base')
% ����������������嵽�����ˡ��ڵ���addBody��������ʱ��ָ����ǰ���������ơ�ÿ���̶��任�����ǰһ���ؽ�����ϵ��
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

showdetails(robot)

conf= homeConfiguration(robot);
conf(2).JointPosition=pi/2;

show(robot,conf);
axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])


























