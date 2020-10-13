%Create Joint-Space Motion Model
clear all
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
tspan = 0:0.01:2;
initialState = [homeConfiguration(robot); zeros(7,1)];
targetState = [pi/4; pi/3; pi/2; -pi/3; pi/4; -pi/4; 3*pi/4; zeros(7,1); zeros(7,1)];
motionModel = jointSpaceMotionModel("RigidBodyTree",robot);

%注意下面都是闭环误差动力学方程
motionModel.NaturalFrequency %误差动力学固有频率，在设置"ComputedTorqueControl" or "IndependentJointMotion".下有效
motionModel.DampingRatio %二阶误差动力学的阻尼比，指定为实值的标量或n元素向量
motionModel.Kp %在PD模型控制下有效
motionModel.Kd

% 运动类型，指定为字符串标量或字符向量，定义对象建模的闭循环联合空间行为。选项有:
% “ComputedTorqueControl”-补偿全身动力学，并分配在自然频率和DampingRatio属性中指定的误差动力学。（默认模式）（需要参考位置速度加速度）
% “独立关节运动”——使用自然频率和阻尼特性指定的误差动力学，将每个关节建模为一个独立的二阶系统。（需要参考位置速度加速度）
% “PDControl”-根据指定的Kp和Kd属性对关节进行比例微分控制。这里应该就是电机PD控制，和书上PD力矩控制不同（只需要参考位置速度）

%通过计算转矩控制和由5%超调的中等快速阶跃响应定义的误差动力学，对系统进行建模。
updateErrorDynamicsFromStep(motionModel,.3,.05);%设置成0就不超调，这个函数是动力学转矩控制

[t,robotState] = ode45(@(t,state) derivative(motionModel,state,targetState),tspan,initialState);
%@(t,state)derivative, t state作为odefun-der()函数的入参,state是current state当前机器人状态
%匿名函数使用https://ww2.mathworks.cn/help/matlab/math/parameterizing-functions.html
%@(t,state) derivative(motionModel,state,targetState)
%意思是motionModel,targetState是derivative函数的入参数，这俩个参数是在外层设置的与der函数自己有关的参数，不是ode计算的入参变量
%ode函数 https://ww2.mathworks.cn/help/matlab/ref/ode15s.html#bu5yps7_sep_shared-odefun
%ode(@(t,state))这部分是ode计算的入参，同时der函数中也存在state，所以这样才能计算微分

figure
plot(t,robotState(:,1:motionModel.NumJoints)); %1-7列关节位置
hold all;
plot(t,targetState(1:motionModel.NumJoints)*ones(1,length(t)),"--"); %参考目标关节位置
title("Joint Position (Solid) vs Reference (Dashed)");
xlabel("Time (s)")
ylabel("Position (rad)");


%Create Task-Space Motion Model
clear all
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
tspan = 0:0.001:1;
initialState = [homeConfiguration(robot);zeros(7,1)];
refPose = trvec2tform([0.6 -.1 0.5]);
refVel = zeros(6,1);
motionModel = taskSpaceMotionModel("RigidBodyTree",robot,"EndEffectorName","EndEffector_Link");
%运动类型，指定为“PDControl”，它使用比例导数(PD)控制(笛卡尔空间)映射到关节通过一个雅克比转置控制器。该控件基于指定的Kp和Kd属性。
motionModel.MotionType
%使用硬解算器在1秒内模拟行为，以更有效地捕获机器人动态。使用ode15s可以在高变化率的区域周围实现更高的精度。
%注意计算出的速度并不是期望末速度0？
[t,robotState] = ode15s(@(t,state)derivative(motionModel,state,refPose,refVel),tspan,initialState);
%ode15s求解刚性微分方程和 DAE - 变阶方法，
%derivative机械手模型状态的时间导数
%所有 MATLAB? ODE 求解器都可以解算 y′=f(t,y) 形式的方程组，或涉及质量矩阵 M(t,y)y′=f(t,y) 的问题。
%求解器都使用类似的语法。ode23s 求解器只能解算质量矩阵为常量的问题。
%ode15s 和 ode23t 可以解算具有奇异质量矩阵的问题，称为微分代数方程 (DAE)。使用 odeset 的 Mass 选项指定质量矩阵。
%state关节位置和速度表示为一个1*2n元向量，表示为[q;qDot]。n是motionModel的关联刚体树中的关节数。表示每个关节的位置，单位为弧度。qDot表示每个关节的速度，单位为弧度/秒。
figure
show(robot,initialState(1:7));
hold all
plot3(refPose(1,4),refPose(2,4),refPose(3,4),"x","MarkerSize",20)

r = rateControl(100);%观察机器人在5hz回路中的响应。
for i = 1:size(robotState,1)
    show(robot,robotState(i,1:7)',"PreservePlot",false);
    waitfor(r);  %暂停代码执行以达到预期的执行速度
end

%%%%%%%%%%%%
%注意derivative 返回的速度，也就是说ode是通过dy/dt=[]微分方程计算y，正好der就是dy/dt。













