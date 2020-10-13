% lesson_9_ConstraintInputs 广义逆运动学约束

% constraintCartesianBounds对象描述了一个物体(末端执行器)相对于固定在另一个物体(参考物体)上的目标坐标系的位置的约束。
% 如果末端执行器原点相对于目标坐标系的位置保持在指定的范围内，则满足此约束。
% TargetTransform属性是将目标坐标系中的点转换为ReferenceBody坐标系中的点的齐次转换。如果是同一个基础坐标系，那么就是对角阵。
% 约束对象用于广义逆运动学对象，以指定机器人的多个运动学约束。
% 末端执行器位置相对于目标坐标系的界限，指定为一个3×2的向量，[xMin xMax;yMin yMax;zMin zMax]。
% 每一行分别定义xyz坐标的最小值和最大值。指定末端执行器相对于全局（目标坐标系，默认全局）的上下限。
% example
% heightAboveTable =
% constraintCartesianBounds(gripper);%ReferenceBody=[]''默认是参考基座。

%constraintOrientationTarget对物体的相对方向进行约束
% constraintOrientationTarget对象描述了一个约束，该约束要求一个物体(末端执行器)的方向在任何方向的角公差内与目标方向匹配。
% 目标方向是相对于参考体的体框架指定的。
%orientationConst = constraintOrientationTarget(endeffector)返回一个方向目标对象，该对象表示由endeffector指定的机器人模型上的一个约束。

% constraintPositionTarget对象描述了一个约束，该约束要求一个物体(末端执行器)的位置在任何方向上到目标位置的距离误差满足约束。
% 目标位置是相对于参考体的主体框架指定的（比如机器人基坐标系）。约束对象用于广义逆运动学对象，以指定机器人的多个运动学约束。

% 这个例子展示了如何使用广义逆运动学来规划机器人的关节空间轨迹。它结合了多种约束条件来生成一个轨迹，引导握柄指向放在桌子上的杯子。
% 这些约束确保了握杯器以直线接近杯子，并且握杯器保持与桌子的安全距离，而不需要事先确定握杯器的姿势。
% 这个例子的目的是生成一系列的机器人配置满足下列标准:
% 在home开始配置没有突然变化的机器人配置
% 保持爪至少5厘米以上“桌子”(z = 0)。
% 当杯子靠近时，夹具应与杯子对齐
% 夹具最后用距离杯子中心5厘米的夹持器完成
% 这个例子利用约束对象生成机器人配置满足这些标准。生成的轨迹由五个配置路径点组成。
% 第一个路点q0被设置为主配置。使用repmat在qwaypoint中预先分配其余配置。
clear
%加载iiwa 14kg urdf模型
lbr = importrobot('iiwa14.urdf'); 
lbr.DataFormat = 'row'; %数据格式行向量，q=[q1 q2 q3 q4 q5 q6 q7]
gripper = 'iiwa_link_ee_kuka';  %夹具名称

%杯子三维数据
cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [-0.5, 0.5, cupHeight/2];

%将一个固定的物体添加到代表杯子中心的机器人模型中。
body = rigidBody('cupFrame');%增加一个刚体坐标系
setFixedTransform(body.Joint, trvec2tform(cupPosition))
addBody(lbr, body, lbr.BaseName);%杯子body添加到lbr的base

numWaypoints = 5;
q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, numWaypoints, 1);
% 创建一个通用的逆向运动学求解器，接受以下约束输入:
% 直角界限-限制夹持器的高度
% 位置目标——指定杯子相对于钳子的位置。
% 瞄准约束-使夹持器与杯轴对齐
% 定位目标-保持一个固定的方向，为夹具，而接近的杯子
% 关节位置界限-限制在路线点之间联合位置的变化。
gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'});

%创建一个笛卡尔边界约束，要求夹具至少在桌子上方5厘米。所有其他值都用inf或-inf表示。
heightAboveTable = constraintCartesianBounds(gripper); %对gripper进行位置约束，在base的z方向上方至少5CM
heightAboveTable.TargetTransform %gripper原点是在全局下的点
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.05, inf];
% 创建一个相对于夹具的杯的位置约束，公差为5毫米。
% constraintPositionTarget对象描述了一个约束，该约束要求一个物体(末端执行器)的位置在任何方向的距离公差内匹配目标位置。
% 目标位置是相对于参考体的主体框架指定的。
distanceFromCup = constraintPositionTarget('cupFrame');%要求杯子坐标系位置限制
distanceFromCup.ReferenceBody = gripper;% 参考坐标系是抓手
distanceFromCup.PositionTolerance = 0.005 ;% 距离误差是5mm
%找到一个指向杯子的结构
%这种配置应该将夹头放置在离杯一定距离的地方，这样最后的进近就可以使夹头正确对齐。
intermediateDistance = 0.3;
%设置杯子目标位置在夹具坐标系下。在规定的距离内，杯体应位于夹持器的z轴上。
distanceFromCup.TargetPosition = [0,0,intermediateDistance];%注意上面设置的参考坐标系是gripper=iiwa_link_ee_kuka在他的Z轴上

% 创建一个瞄准约束，要求iiwa_link_ee坐标系的z轴近似垂直，方法是将目标放置在机器人上方很远的地方,因为一直会调用很近的话会改变姿态。
% iiwa_link_ee坐标系是定向的，因此这个约束将夹持器与杯轴对齐。
alignWithCup = constraintAiming('iiwa_link_ee');%注意这里是ee不是ee_kuka，他的Z轴是水平的
alignWithCup.TargetPoint = [0, 0, 100];

%创建一个关节位置边界约束。根据前面的配置设置此约束的Bounds属性，以限制关节位置的变化。
limitJointChange = constraintJointBounds(lbr);
%为夹持器创建一个方向约束，公差为1度。此约束要求夹持器的方向与TargetOrientation属性指定的值匹配。
%使用此约束来固定在最后接近杯子时夹持器的方向。 注意是最后

%所述末端执行器相对于所述参考体框架的方向是将所述末端执行器框架中指定的方向转换为所述参考体框架中指定的相同方向的方向。
%将末端执行器框架中指定的方向转换为与参考体框架中指定的方向相同。
%此约束要求夹持器gripper的方向与TargetOrientation属性指定的值匹配。使用此约束来固定在最后接近杯子时夹持器的方向。
fixOrientation = constraintOrientationTarget(gripper);
fixOrientation.OrientationTolerance = deg2rad(1);

%约束对象有一个权值属性，它决定了求解器如何处理冲突约束。将约束的权值设置为0将禁用该约束。对于此配置，禁用关节位置界限和方向约束。
limitJointChange.Weights = zeros(size(limitJointChange.Weights));
fixOrientation.Weights = 0;%将约束的权值设置为0将禁用该约束，也就是2点根本没用这个约束！。对于此配置，禁用关节位置界限和方向约束。

%使用gik求解器求解满足输入约束的机器人配置。必须指定所有输入约束。将该配置设置为第二个路点。
[qWaypoints(2,:),solutionInfo] = gik(q0, heightAboveTable, ...
                       distanceFromCup, alignWithCup, fixOrientation, ...
                       limitJointChange);

%找到将夹具沿直线移动到杯中的配置，重新启用关节位置约束和方向约束。
limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;%启用约束！
%禁用“与杯对齐”约束，因为方向约束使其冗余。
alignWithCup.Weights = 0; %禁用约束！

%根据前面的配置设置方向约束来保持方向(qWaypoints(2，:))。得到从夹持器到机器人模型基础的变换。将齐次变换转换为四元数。
fixOrientation.TargetOrientation = ...
tform2quat(getTransform(lbr,qWaypoints(2,:),gripper));

%为每个路径点定义杯子和夹具之间的距离
finalDistanceFromCup = 0.05;
distanceFromCupValues = linspace(intermediateDistance, finalDistanceFromCup, numWaypoints-1);

%定义每个运行点之间的关节位置允许的最大变化。
maxJointChange = deg2rad(10);

%为每个剩余的路点调用求解程序。
for k = 3:numWaypoints
    % 更新目标位置。
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k-1);
    % 将关节位置限制在其先前值附近。
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    % 解决配置问题并将其添加到waypoints数组中。
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         heightAboveTable, ...
                                         distanceFromCup, alignWithCup, ...
                                         fixOrientation, limitJointChange);
end

%可视化生成的轨迹，在路径点之间插入以生成平滑的轨迹。使用pchip避免过冲，可能会违反机器人的关节极限。
framerate = 15;
r = rateControl(framerate);
tFinal = 10; %运行10S
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];%5个路径点时间信息
numFrames = tFinal*framerate; %一共150真，相当于插150个点
%分段三次 Hermite 插值多项式 (PCHIP)
%p = pchip(x,y,xq) 返回与 xq 中的查询点对应的插值 p 的向量。p 的值由 x 和 y 的保形分段三次插值确定。
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';%150组关节角

gripperPosition = zeros(numFrames,3); %夹具 XYZ
for k = 1:numFrames
    gripperPosition(k,:) = tform2trvec(getTransform(lbr,qInterp(k,:), ...
                                                    gripper));
end

figure;
show(lbr, qWaypoints(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
p = plot3(gripperPosition(1,1), gripperPosition(1,2), gripperPosition(1,3));

hold on
for k = 1:size(qInterp,1)
    show(lbr, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k,1);
    p.YData(k) = gripperPosition(k,2);
    p.ZData(k) = gripperPosition(k,3);
    waitfor(r);
end
hold off