%lesson 2 rigidBodyTree
% �������Ǹ�����ؽ������Ե�һ�ֱ�ʾ��ʹ���������MATLAB?�н��������˻�е��ģ�͡��������һ��ʹ��ͳһ������������ʽ(URDF)ָ���Ļ�����ģ�ͣ���ʹ��importrobot���������Ļ�����ģ�͡�
% ������ģ���ɸ�����Ϊ���������ɡ�ÿ�����嶼��һ����֮��ص�rigidBodyJoint���󣬸ö����������������丸������ƶ���ʹ��setFixedTransform������ؽڵĿ������������Ŀ��֮��Ĺ̶�ת����
% ������ʹ��RigidBodyTree��ķ�����ģ������ӡ��滻��ɾ�����塣
% �����˶���ѧ����Ҳ�ǿ��ܵġ�ָ��������ģ����ÿ����������������ĺ͹������ԡ�����Լ����л�û������������ͷ�����ѧ����������������˹ؽ��˶��͹ؽ�����Ķ���ѧ����
% Ҫʹ���붯̬��صĺ������뽫DataFormat��������Ϊ��row����column����
% ���ڸ����ĸ�����ģ�ͣ�������ʹ�û��������˶�ѧ�㷨ʹ�û�����ģ�ͼ�������ĩ��ִ����λ�õĹؽڽǡ���ʹ�����˶�ѧ��������˶�ѧʱ��ָ����ĸ�����ģ�͡�
% ��ʾ����֧��������Ŀ��ӻ�������ָ��Ϊ.stl�ļ�������ʹ��addVisual��ӵ������ĸ����С����⣬Ĭ������£�importrobot����������URDF������ģ����ָ�������пɷ��ʵ�.stl�ļ���
% ����һ�����νṹ�Ļ����˶���ʹ��addBody��Ӹ��塣
% robot = rigidBodyTree("MaxNumBodies"��N��"DataFormat"�� DataFormat)ָ�������ɴ���ʱ�����������body���������ޡ��������뽫DataFormat����ָ��Ϊ����-ֵ�ԡ�

% ����
% �ڸ������м���������Ӧ�Ĺؽڡ�ÿ��rigidBody���󶼰���һ��rigidBodyJoint���󣬱���ʹ��addBody������ӵ�rigidBodyTree�С�
% �������е�rigidBodyTree����������ڸ��������滻�ؽڣ����������������lesson1ʾ����������ΪrigidBodyTree����
% ��ȡһ���ض���������������ԡ�L3���Ψһ��Ԫ����L4�塣��Ҳ���Ը����ض������塣
clear
load exampleRobots.mat
showdetails(puma1)
% ��ȡrigidtree������ΪL3������
body3 = getBody(puma1,'L3'); 
% ��ȡһ���ض���������������ԡ�L3���Ψһ��Ԫ����L4�塣��Ҳ���Ը����ض������塣
childBody = body3.Children{1} 
body3Copy = copy(body3) %��Ȼ�Ǹ���L3���ǲ����������Ӹ�����
% ����L3�Ĺؽ����Բ��ı������������봴��һ���µĹؽڶ��󣬲�ʹ��replaceJoint��ȷ�����ε����弸����״����Ӱ�졣
% �����Ҫ��������֮���ת��������ʹ��Ĭ�ϵı�ʶ���󣬿��Ե���setFixedTransform��
newJoint = rigidBodyJoint('prismatic');%�ƶ��ؽ�
replaceJoint(puma1,'L3',newJoint);%�滻L3�Ĺؽ�Ϊ�¹ؽ�

showdetails(puma1)
%ɾ������body��ʹ��removeBody��ý��������ɾ������������������С�
subtree = removeBody(puma1,'L4')

%�Ƴ��޸ĺ��L3���塣��ԭʼ���Ƶ�L3������ӵ�L2���壬Ȼ�󷵻�������
%������ģ�ͱ��ֲ��䡣ͨ��showdetails�鿴��ϸ�ıȽϡ�
removeBody(puma1,'L3');
addBody(puma1,body3Copy,'L2')%L2�Ǹ�
addSubtree(puma1,'L3',subtree)%����֮ǰ����������L3

showdetails(puma1)

% ָ���������Ķ���ѧ����
% Ҫʹ�ö���ѧ����������ؽ�Ť�غͼ��ٶȣ���ָ��rigidBodyTree�����rigidBody�Ķ���ѧ���ԡ�
% ����һ��������ģ�͡������������帽���������档
clear
robot = rigidBodyTree('DataFormat','row');
body1 = rigidBody('body1');
body2 = rigidBody('body2');
% ָ�����ӵ������ϵĹؽڡ���body2�Ĺ̶�ת������Ϊbody1������任��x��������1m��
joint1 = rigidBodyJoint('joint1','revolute');
joint2 = rigidBodyJoint('joint2');
setFixedTransform(joint2,trvec2tform([1 0 0]))
body1.Joint = joint1;
body2.Joint = joint2;
% ָ����������Ķ���ѧ���ԡ���������ӵ�������ģ���С�
% ���ڱ����������˴��и�����������(body2)�ĸ�(body1)�Ļ���ֵ��

body1.Mass = 2;
body1.CenterOfMass = [0.5 0 0];
body1.Inertia = [0.167 0.001 0.167 0 0 0];

body2.Mass = 1;
body2.CenterOfMass = [0 0 0];
body2.Inertia = 0.0001*[4 4 4 0 0 0];

addBody(robot,body1,'base');
addBody(robot,body2,'body1');

%�������������˵�����λ�á����������˵�λ�á�����ͼ�ƶ���xyƽ�档
comPos = centerOfMass(robot);

show(robot);
hold on
plot(comPos(1),comPos(2),'or')
view(2)

%�ı�ڶ��������������ע�����ĵı仯��
body2.Mass = 20;
replaceBody(robot,'body2',body2) %ע��������ֱ���滻����

comPos2 = centerOfMass(robot);
plot(comPos2(1),comPos2(2),'*g')
hold off

%�ڸ�����ģ���ϼ����������������������ѧ����֪������λ�ã�
%һ�����ֱ�Ӧ�õ�һ���ض���������������ָ��Ϊ���������ˡ�����Ԥ�����KUKA LBR������ģ�ͣ���ģ��ָ��ΪRigidBodyTree����
clear
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.81];
q = homeConfiguration(lbr);
q(2)=pi/4
% ָ����ʾ�������������������İ���������������������������������
% ָ��������ģ�͡�������ֵ�ĩ��ִ����������ʸ���͵�ǰ���������á�
% ����������ڡ�tool0�������ܸ����ģ�Ҳ������tool0����ϵʩ�ӵģ���Ҫ����ָ�������˵����ã�q��
wrench = [0 0 0.5 0 0 0.3];
%externalForce(robot,bodyname,wrench)�������������������ʹ�øþ�����Ϊ������ѧ��������ѧ�����룬
%��bodynameָ���ĸ���ʩ��һ���������֡��ٶ����������ڻ�������ڣ�������fext�ڻ�����ϵ�и�����
% ʩ���������ϵ����غ�����ָ��Ϊ[Tx Ty Tz Fx Fy Fz]ʸ�������ֵ�ǰ����Ҫ�ض�Ӧ��xyz����Χ�����ء�
% �������Ԫ������ͬһ�����������������ָ���˻����˵����ã����ĸ�Ԫ��q��������������������base��
fext = externalForce(lbr,'tool0',wrench,q);
% ���㵱lbr����������ʱ��ĩ��ִ������tool0�����ܵ����������������²����Ĺؽڼ��ٶȡ�
% ����ؽ��ٶȺ͹ؽ�����Ϊ��(����Ϊ������[])��
qddot = forwardDynamics(lbr,q,[],[],fext);


%����ؽ�Ť���ԶԿ�����
% ����������������������Ӧ���ڸ�����ģ�͡���������һ��m��6�����������ڻ������ϵ�ÿ���ؽڶ���һ����Ӧ��һ����Ԫ���֡�
% ʹ������������ָ��ĩ��ִ��������ȷ�ؽ����ַ��䵽�������ȷ�С�����Խ�������������һ������һ��������ʩ�Ӷ������
% Ҫ���������Щ�����Ĺؽ�Ť�أ�����ʹ�÷�����ѧ������
% ����һ��Ԥ�����KUKA LBR������ģ�ͣ�����ָ��ΪRigidBodyTree
clear
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.81];
q = homeConfiguration(lbr);
%��link1����������������İ��������ڻ�����ϵ�б�ʾ��
fext1 = externalForce(lbr,'link_1',[0 0 0.0 0.1 0 0]);
%��ĩ��ִ����������������tool0������İ���������tool0�б�ʾ��
fext2 = externalForce(lbr,'tool0',[0 0 0.0 0.1 0 0],q);
%fext1 fext2���ػ���ȫ������ϵ������Ȼ����ʾ���ĸ�����ϵ�е������ã���ֵ�ǻ���ȫ�ֵġ�
%����ƽ����������Ĺؽ����ء�Ҫ�����Щ��������������ӡ�����ؽ��ٶȺͼ��ٶ�Ϊ��(����Ϊ[])��
tau = inverseDynamics(lbr,q,[],[],fext1+fext2);
%jointTorq = inverseDynamics(robot) ������ѧ(������)����������ڲ�ʩ������������¾�ֹ��������ṹ����Ĺؽ����ء�


