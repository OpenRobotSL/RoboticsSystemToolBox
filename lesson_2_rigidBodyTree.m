%lesson 2 rigidBodyTree
% 刚体树是刚体与关节连接性的一种表示。使用这个类在MATLAB?中建立机器人机械手模型。如果您有一个使用统一机器人描述格式(URDF)指定的机器人模型，请使用importrobot来导入您的机器人模型。
% 刚体树模型由刚体作为刚体对象组成。每个刚体都有一个与之相关的rigidBodyJoint对象，该对象定义了如何相对于其父体进行移动。使用setFixedTransform来定义关节的框架与相邻物体的框架之间的固定转换。
% 您可以使用RigidBodyTree类的方法从模型中添加、替换或删除刚体。
% 机器人动力学计算也是可能的。指定机器人模型中每个刚体的质量、重心和惯性特性。你可以计算有或没有外力的正向和反向动力学，并计算给定机器人关节运动和关节输入的动力学量。
% 要使用与动态相关的函数，请将DataFormat属性设置为“row”或“column”。
% 对于给定的刚体树模型，还可以使用机器人逆运动学算法使用机器人模型计算所需末端执行器位置的关节角。当使用逆运动学或广义逆运动学时，指定你的刚体树模型。
% 显示方法支持体网格的可视化。网格被指定为.stl文件，可以使用addVisual添加到单独的刚体中。另外，默认情况下，importrobot函数将加载URDF机器人模型中指定的所有可访问的.stl文件。
% 创建一个树形结构的机器人对象。使用addBody添加刚体。
% robot = rigidBodyTree("MaxNumBodies"，N，"DataFormat"， DataFormat)指定在生成代码时机器人允许的body数量的上限。您还必须将DataFormat属性指定为名称-值对。

% 例程
% 在刚体树中加入刚体和相应的关节。每个rigidBody对象都包含一个rigidBodyJoint对象，必须使用addBody将其添加到rigidBodyTree中。
% 更改现有的rigidBodyTree对象。你可以在刚体树中替换关节，物体和子树。加载lesson1示例机器人作为rigidBodyTree对象。
% 获取一个特定的主体来检查属性。L3体的唯一子元素是L4体。您也可以复制特定的主体。
clear
load exampleRobots.mat
showdetails(puma1)
% 获取rigidtree中名字为L3刚体句柄
body3 = getBody(puma1,'L3'); 
% 获取一个特定的主体来检查属性。L3体的唯一子元素是L4体。您也可以复制特定的主体。
childBody = body3.Children{1} 
body3Copy = copy(body3) %虽然是复制L3但是不复制他的子父属性
% 更换L3的关节属性不改变其他。您必须创建一个新的关节对象，并使用replaceJoint来确保下游的人体几何形状不受影响。
% 如果需要定义主体之间的转换而不是使用默认的标识矩阵，可以调用setFixedTransform。
newJoint = rigidBodyJoint('prismatic');%移动关节
replaceJoint(puma1,'L3',newJoint);%替换L3的关节为新关节

showdetails(puma1)
%删除整个body并使用removeBody获得结果子树。删除的主体包含在子树中。
subtree = removeBody(puma1,'L4')

%移除修改后的L3刚体。将原始复制的L3刚体添加到L2刚体，然后返回子树。
%机器人模型保持不变。通过showdetails查看详细的比较。
removeBody(puma1,'L3');
addBody(puma1,body3Copy,'L2')%L2是父
addSubtree(puma1,'L3',subtree)%加载之前子树，父是L3

showdetails(puma1)

% 指定刚体树的动力学特性
% 要使用动力学函数来计算关节扭矩和加速度，请指定rigidBodyTree对象和rigidBody的动力学特性。
% 创建一个刚体树模型。创造两个刚体附着在它上面。
clear
robot = rigidBodyTree('DataFormat','row');
body1 = rigidBody('body1');
body2 = rigidBody('body2');
% 指定连接到主体上的关节。将body2的固定转换设置为body1。这个变换在x方向上是1m。
joint1 = rigidBodyJoint('joint1','revolute');
joint2 = rigidBodyJoint('joint2');
setFixedTransform(joint2,trvec2tform([1 0 0]))
body1.Joint = joint1;
body2.Joint = joint2;
% 指定两个物体的动力学特性。将身体添加到机器人模型中。
% 对于本例，给出了带有附加球形质量(body2)的杆(body1)的基本值。

body1.Mass = 2;
body1.CenterOfMass = [0.5 0 0];
body1.Inertia = [0.167 0.001 0.167 0 0 0];

body2.Mass = 1;
body2.CenterOfMass = [0 0 0];
body2.Inertia = 0.0001*[4 4 4 0 0 0];

addBody(robot,body1,'base');
addBody(robot,body2,'body1');

%计算整个机器人的质心位置。画出机器人的位置。将视图移动到xy平面。
comPos = centerOfMass(robot);

show(robot);
hold on
plot(comPos(1),comPos(2),'or')
view(2)

%改变第二个物体的质量。注意重心的变化。
body2.Mass = 20;
replaceBody(robot,'body2',body2) %注意这里是直接替换刚体

comPos2 = centerOfMass(robot);
plot(comPos2(1),comPos2(2),'*g')
hold off

%在刚体树模型上计算由于外力引起的正向动力学（已知力矩求位置）
%一个扳手被应用到一个特定的身体与重力是指定为整个机器人。加载预定义的KUKA LBR机器人模型，该模型指定为RigidBodyTree对象。
clear
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.81];
q = homeConfiguration(lbr);
q(2)=pi/4
% 指定表示机器人所经历的外力的扳手向量。利用外力函数生成外力矩阵。
% 指定机器人模型、体验扳手的末端执行器、扳手矢量和当前机器人配置。
% 扳手是相对于“tool0”车身框架给出的，也就是在tool0坐标系施加的，它要求你指定机器人的配置，q。
wrench = [0 0 0.5 0 0 0.3];
%externalForce(robot,bodyname,wrench)组成了外力矩阵，您可以使用该矩阵作为反向动力学和正向动力学的输入，
%向bodyname指定的刚体施加一个外力扳手。假定扳手输入在基础框架内，力矩阵fext在基坐标系中给出。
% 施加在物体上的力矩和力，指定为[Tx Ty Tz Fx Fy Fz]矢量。扳手的前三个要素对应于xyz轴周围的力矩。
% 最后三个元素是沿同一轴的线性力。除非你指定了机器人的配置，第四个元素q，否则扳手力矩是相对于base。
fext = externalForce(lbr,'tool0',wrench,q);
% 计算当lbr处于主构型时，末端执行器“tool0”所受的外力在重力作用下产生的关节加速度。
% 假设关节速度和关节力矩为零(输入为空向量[])。
qddot = forwardDynamics(lbr,q,[],[],fext);


%计算关节扭矩以对抗外力
% 利用外力函数生成力矩阵，应用于刚体树模型。力矩阵是一个m×6的向量，对于机器人上的每个关节都有一行来应用一个六元扳手。
% 使用外力函数，指定末端执行器，正确地将扳手分配到矩阵的正确行。你可以将多个力矩阵加在一起来对一个机器人施加多个力。
% 要计算抵消这些外力的关节扭矩，可以使用反动力学函数。
% 加载一个预定义的KUKA LBR机器人模型，它被指定为RigidBodyTree
clear
load exampleRobots.mat lbr
lbr.DataFormat = 'row';
lbr.Gravity = [0 0 -9.81];
q = homeConfiguration(lbr);
%在link1上设置外力。输入的扳手向量在基坐标系中表示。
fext1 = externalForce(lbr,'link_1',[0 0 0.0 0.1 0 0]);
%在末端执行器上设置外力，tool0。输入的扳手向量在tool0中表示。
fext2 = externalForce(lbr,'tool0',[0 0 0.0 0.1 0 0],q);
%fext1 fext2返回基于全局坐标系的力，然后显示是哪个坐标系中的力作用，数值是基于全局的。
%计算平衡外力所需的关节力矩。要组合这些力，将力矩阵相加。假设关节速度和加速度为零(输入为[])。
tau = inverseDynamics(lbr,q,[],[],fext1+fext2);
%jointTorq = inverseDynamics(robot) 反动力学(机器人)计算机器人在不施加外力的情况下静止保持自身结构所需的关节力矩。


