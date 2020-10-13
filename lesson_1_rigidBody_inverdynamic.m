% lesson 1 rigidBody
% �������κ����ͻ�е�۵Ļ���������ÿ�������϶���һ������ؽڶ��󣬸ö������˸�����˶���ʽ��
% ���ø�����������װ���һ�����ͽṹ�Ļ�����ģ�͡��ڵ���addBody��������ӵ�������ģ��֮ǰ����joint��������Ϊjoint���ԡ�
% �������ڸ������У��㲻��ֱ���޸��������ԣ���Ϊ���ƻ�������֮��Ĺ�ϵ��ʹ��replaceJoint���޸��������ṹ��
% ��������ָ�����Ƶĸ��塣Ĭ������£�������һ���̶��Ĺؽڡ�

% Examples ������ͽ�ͷ���ӵ���������

clear

rbtree = rigidBodyTree('DataFormat', 'row')

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
dhparams = [0   	0	    0   	0;
            0.32	pi/2    0       0
            0.975	0	0	    0;
            0.2   	pi/2	0.887	0;
            0       -pi/2	0   	0;
            0       pi/2       0       0];

robot = rigidBodyTree('DataFormat', 'row');
robot.Gravity = [0 0 -9.81];
% ����һ��rigidBody���󲢸���һ��Ψһ�����ơ�
body1 = rigidBody('body1'); %��һ������
jnt1 = rigidBodyJoint('jnt1','revolute');%��������Ĺؽ���ת���ؽ����ֽ�jnt1
jnt1.JointAxis = [0 0 1];
% ʹ��setFixedTransformָ��ʹ��DH�����Ĵ����嵽�����ת����DH���������һ��Ԫ�أ��������ˣ���Ϊ�Ƕ������ڹؽڵ�λ�á�
setFixedTransform(jnt1,dhparams(1,:),'mdh');%����DH�任���Ӹ����Լ��������������dh��mdh

body1.Joint = jnt1;%�ѹؽ����Դ�������

% ����������������嵽�����ˡ��ڵ���addBody��������ʱ��ָ����ǰ���������ơ�ÿ���̶��任�����ǰһ���ؽ�����ϵ��
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.JointAxis = [0 0 1];
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
jnt3.JointAxis = [0 0 1];
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
jnt4.JointAxis = [0 0 1];
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
jnt5.JointAxis = [0 0 1];
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
jnt6.JointAxis = [0 0 1];

setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

body1.Mass = 7.36;
body2.Mass = 36.26;
body3.Mass = 12.44;
body4.Mass = 1.21;
body5.Mass = 1.21;
body6.Mass = 1;

body1.CenterOfMass = 0.5 * jnt2.JointToParentTransform(1:3,4);
body2.CenterOfMass = 0.5 * jnt3.JointToParentTransform(1:3,4);
body3.CenterOfMass = 0.5 * jnt4.JointToParentTransform(1:3,4);
body4.CenterOfMass = 0.5 * jnt5.JointToParentTransform(1:3,4);
body5.CenterOfMass = 0.5 * jnt6.JointToParentTransform(1:3,4);
body6.CenterOfMass = 0.5 * [0 0 0];
body1.Inertia = [0 0 0.6 0 0 0];
body2.Inertia = [0 0 0.5 0 0 0];
body3.Inertia = [0 0 0.4 0 0 0];
body4.Inertia = [0 0 0.3 0 0 0];
body5.Inertia = [0 0 0.2 0 0 0];
body6.Inertia = [0 0 0.1 0 0 0];
% ����addBody����һ������ؽ����ӵ������˵Ļ�������ϡ�
addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

showdetails(robot)

conf= homeConfiguration(robot);
%conf(1).JointPosition=0;
%conf(2).JointPosition=pi/6;
%conf(3).JointPosition=0;
%conf(4).JointPosition=0;
%conf(5).JointPosition=pi/6;
%conf(6).JointPosition=0;

%show(robot,conf);
%axis([-0.5,0.5,-0.5,0.5,-0.5,0.5])

disp(jnt1.JointToParentTransform)
disp(jnt2.JointToParentTransform)
disp(jnt3.JointToParentTransform)
disp(jnt4.JointToParentTransform)
disp(jnt5.JointToParentTransform)
disp(jnt6.JointToParentTransform)
disp(robot.geometricJacobian(conf, 'body6'))
disp(robot.getTransform(conf, 'body6'))
disp(pinv(robot.geometricJacobian(conf, 'body6')))

q = [0, pi/4, pi/4, pi/4, pi/3, 0];
show(robot,q);
qdot = [1 1 2 2 1 0];
qdotdot = [0 0 0 0 0 0];
fext = [0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;
        0 0 0 0 0 0;]
tau = robot.inverseDynamics(q, qdot, qdotdot, []);
inverseDynamics(robot,q,qdot,[],[]);
disp(tau)

transform = getTransform(robot,[0, pi/4, pi/4, pi/4, pi/3, 0],'body6')










