% lesson 1 rigidBody
% 刚体是任何树型机械臂的基本构件。每个刚体上都有一个刚体关节对象，该对象定义了刚体的运动方式。
% 利用刚体树将刚体装配成一个树型结构的机器人模型。在调用addBody将刚体添加到机器人模型之前，将joint对象设置为joint属性。
% 当刚体在刚体树中，你不能直接修改它的属性，因为它破坏了物体之间的关系。使用replaceJoint来修改整个树结构。
% 创建具有指定名称的刚体。默认情况下，身体有一个固定的关节。

% Examples 将刚体和接头连接到刚体树上

clear

rbtree = rigidBodyTree('DataFormat', 'row')

% 创建一个具有唯一名称的刚体。默认关节属性是固定
body1 = rigidBody('b1')
% 针对上面创建好的刚体，设置刚体的关节属性和名称，rigidBodyJoint就是上面rigidBody中的Joint属性
jnt1 = rigidBodyJoint('jnt1','revolute')
body1.Joint = jnt1
% 将刚体添加到树中。指定要将刚体附加到的主体名称。因为这是第一个主体，所以使用树的基名。
basename = rbtree.BaseName%首先获取树中base的名称，然后把创建的rigid加入到树中，base是父rigid
addBody(rbtree,body1,basename)
% 使用树的showdetails来确认刚体和关节是否正确添加
showdetails(rbtree)

% 利用Denavit-Hartenberg参数构建机械手机器人
% 使用Puma560?机器人的Denavit-Hartenberg (DH)参数构建机器人。每个刚体一次添加一个，由关节对象指定子到父的转换。
% DH参数定义了机器人的几何形状，与每个刚体如何附着到其父体有关。为了方便起见，在矩阵中设置Puma560机器人的参数。
% Puma机器人是一个串行链式机械手。DH参数相对于矩阵中的前一行，对应于前一个关节连接。

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
% 创建一个rigidBody对象并给它一个唯一的名称。
body1 = rigidBody('body1'); %第一个刚体
jnt1 = rigidBodyJoint('jnt1','revolute');%刚体上面的关节是转动关节名字叫jnt1
jnt1.JointAxis = [0 0 1];
% 使用setFixedTransform指定使用DH参数的从主体到主体的转换。DH参数的最后一个元素，被忽略了，因为角度依赖于关节的位置。
setFixedTransform(jnt1,dhparams(1,:),'mdh');%设置DH变换，从父到自己。后面可以设置dh和mdh

body1.Joint = jnt1;%把关节属性传给刚体

% 创建并添加其他刚体到机器人。在调用addBody来附加它时，指定以前的主体名称。每个固定变换相对于前一个关节坐标系。
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
% 调用addBody将第一个身体关节连接到机器人的基础框架上。
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










