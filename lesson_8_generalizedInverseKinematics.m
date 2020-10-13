%lesson_8_generalizedInverseKinematics 建立多约束逆运动学求解器
% 广义逆运动学系统对象使用一组运动学约束来计算由刚体对象指定的刚体树模型的关节构型。
% 广义逆运动学对象采用非线性解算器来满足约束条件或达到最佳逼近。
% 在调用对象之前指定约束类型ConstraintInputs。要在调用对象后更改约束输入，请调用release(gik)。
% 将约束输入指定为约束对象，并调用传递这些对象的通用逆运动学。要创建约束对象，请使用以下对象

% 求解一组约束的广义逆运动学
% 创建一个广义逆运动学求解器，将机器人手臂固定在特定位置并指向机器人基座。创建约束对象，将必要的约束参数传递到求解器。

load exampleRobots.mat lbr
gik = generalizedInverseKinematics;
gik.RigidBodyTree = lbr;
% 创建两个约束对象。
gik.ConstraintInputs = {'position','aiming'};
% 名为tool0的物体的原点相对于机器人的base位于[0.0 0.5 0.5]。
posTgt = constraintPositionTarget('tool0');
posTgt.TargetPosition = [0.0 0.5 0.5];
% 名为tool0的刚体z轴指向机器人基础框架的原点。也就是末端tool0的Z轴指向000这个点
aimCon = constraintAiming('tool0');
aimCon.TargetPoint = [0.0 0.0 0.8];
%找到一个满足约束的配置。您必须按照在ConstraintInputs属性中指定的顺序将约束对象传递到系统对象。指定机器人配置的初始猜测。
q0 = homeConfiguration(lbr); % Initial guess for solver
[q,solutionInfo] = gik(q0,posTgt,aimCon);
% solutionInfo解决方案信息，作为结构返回。解决方案信息结构包含以下字段:
% 迭代——算法运行的迭代次数。
% NumRandomRestarts—由于算法陷入局部最小值而导致随机重新启动的次数。
% PoseErrorNorm—解决方案与期望末端执行器姿态相比的位姿误差的大小。
% ExitFlag -代码，提供更多关于算法执行的细节，以及什么导致它返回。有关每种算法类型的退出标志，请参见退出标志。
% 状态-字符向量，描述解决方案是在允许范围内(“成功”)还是算法能够找到的最佳解决方案(“最佳可用”)。
% 可视化求解器返回的配置。
show(lbr,q);
title(['Solver status: ' solutionInfo.Status])
axis([-0.75 0.75 -0.75 0.75 -0.5 1])
% 从目标位置到基地原点绘制一条线段。tool0帧的原点与线段的一端重合，其z轴与线段对齐。
hold on
plot3([0.0 0.0],[0.5 0.0],[0.5 0.8],'--o')
hold off