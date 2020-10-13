%lesson 4 geometricJacobian
% jacobian = geometricJacobian(robot,configuration,endeffectorname)
% 计算相对于指定末端执行器名称和机器人模型配置的基的几何雅可比矩阵。

load exampleRobots.mat puma1

%计算任意构型下Puma机器人“L6”身体的几何雅可比矩阵。

geoJacob = geometricJacobian(puma1,randomConfiguration(puma1),'L6')

% 具有指定配置的末端执行器的几何雅可比矩阵，以6×n矩阵的形式返回，其中n是末端执行器的自由度。
% 雅可比矩阵将关节空间的速度映射到末端执行器的速度，相对于基本坐标系。末端执行器速度等于: