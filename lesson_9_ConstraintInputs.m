% lesson_9_ConstraintInputs �������˶�ѧԼ��

% constraintCartesianBounds����������һ������(ĩ��ִ����)����ڹ̶�����һ������(�ο�����)�ϵ�Ŀ������ϵ��λ�õ�Լ����
% ���ĩ��ִ����ԭ�������Ŀ������ϵ��λ�ñ�����ָ���ķ�Χ�ڣ��������Լ����
% TargetTransform�����ǽ�Ŀ������ϵ�еĵ�ת��ΪReferenceBody����ϵ�еĵ�����ת���������ͬһ����������ϵ����ô���ǶԽ���
% Լ���������ڹ������˶�ѧ������ָ�������˵Ķ���˶�ѧԼ����
% ĩ��ִ����λ�������Ŀ������ϵ�Ľ��ޣ�ָ��Ϊһ��3��2��������[xMin xMax;yMin yMax;zMin zMax]��
% ÿһ�зֱ���xyz�������Сֵ�����ֵ��ָ��ĩ��ִ���������ȫ�֣�Ŀ������ϵ��Ĭ��ȫ�֣��������ޡ�
% example
% heightAboveTable =
% constraintCartesianBounds(gripper);%ReferenceBody=[]''Ĭ���ǲο�������

%constraintOrientationTarget���������Է������Լ��
% constraintOrientationTarget����������һ��Լ������Լ��Ҫ��һ������(ĩ��ִ����)�ķ������κη���Ľǹ�������Ŀ�귽��ƥ�䡣
% Ŀ�귽��������ڲο��������ָ���ġ�
%orientationConst = constraintOrientationTarget(endeffector)����һ������Ŀ����󣬸ö����ʾ��endeffectorָ���Ļ�����ģ���ϵ�һ��Լ����

% constraintPositionTarget����������һ��Լ������Լ��Ҫ��һ������(ĩ��ִ����)��λ�����κη����ϵ�Ŀ��λ�õľ����������Լ����
% Ŀ��λ��������ڲο����������ָ���ģ���������˻�����ϵ����Լ���������ڹ������˶�ѧ������ָ�������˵Ķ���˶�ѧԼ����

% �������չʾ�����ʹ�ù������˶�ѧ���滮�����˵Ĺؽڿռ�켣��������˶���Լ������������һ���켣�������ձ�ָ����������ϵı��ӡ�
% ��ЩԼ��ȷ�����ձ�����ֱ�߽ӽ����ӣ������ձ������������ӵİ�ȫ���룬������Ҫ����ȷ���ձ��������ơ�
% ������ӵ�Ŀ��������һϵ�еĻ����������������б�׼:
% ��home��ʼ����û��ͻȻ�仯�Ļ���������
% ����צ����5�������ϡ����ӡ�(z = 0)��
% �����ӿ���ʱ���о�Ӧ�뱭�Ӷ���
% �о�����þ��뱭������5���׵ļг������
% �����������Լ���������ɻ���������������Щ��׼�����ɵĹ켣���������·������ɡ�
% ��һ��·��q0������Ϊ�����á�ʹ��repmat��qwaypoint��Ԥ�ȷ����������á�
clear
%����iiwa 14kg urdfģ��
lbr = importrobot('iiwa14.urdf'); 
lbr.DataFormat = 'row'; %���ݸ�ʽ��������q=[q1 q2 q3 q4 q5 q6 q7]
gripper = 'iiwa_link_ee_kuka';  %�о�����

%������ά����
cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [-0.5, 0.5, cupHeight/2];

%��һ���̶���������ӵ����������ĵĻ�����ģ���С�
body = rigidBody('cupFrame');%����һ����������ϵ
setFixedTransform(body.Joint, trvec2tform(cupPosition))
addBody(lbr, body, lbr.BaseName);%����body��ӵ�lbr��base

numWaypoints = 5;
q0 = homeConfiguration(lbr);
qWaypoints = repmat(q0, numWaypoints, 1);
% ����һ��ͨ�õ������˶�ѧ���������������Լ������:
% ֱ�ǽ���-���Ƽг����ĸ߶�
% λ��Ŀ�ꡪ��ָ�����������ǯ�ӵ�λ�á�
% ��׼Լ��-ʹ�г����뱭�����
% ��λĿ��-����һ���̶��ķ���Ϊ�оߣ����ӽ��ı���
% �ؽ�λ�ý���-������·�ߵ�֮������λ�õı仯��
gik = generalizedInverseKinematics('RigidBodyTree', lbr, ...
    'ConstraintInputs', {'cartesian','position','aiming','orientation','joint'});

%����һ���ѿ����߽�Լ����Ҫ��о������������Ϸ�5���ס���������ֵ����inf��-inf��ʾ��
heightAboveTable = constraintCartesianBounds(gripper); %��gripper����λ��Լ������base��z�����Ϸ�����5CM
heightAboveTable.TargetTransform %gripperԭ������ȫ���µĵ�
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0.05, inf];
% ����һ������ڼоߵı���λ��Լ��������Ϊ5���ס�
% constraintPositionTarget����������һ��Լ������Լ��Ҫ��һ������(ĩ��ִ����)��λ�����κη���ľ��빫����ƥ��Ŀ��λ�á�
% Ŀ��λ��������ڲο����������ָ���ġ�
distanceFromCup = constraintPositionTarget('cupFrame');%Ҫ��������ϵλ������
distanceFromCup.ReferenceBody = gripper;% �ο�����ϵ��ץ��
distanceFromCup.PositionTolerance = 0.005 ;% ���������5mm
%�ҵ�һ��ָ���ӵĽṹ
%��������Ӧ�ý���ͷ�������뱭һ������ĵط����������Ľ����Ϳ���ʹ��ͷ��ȷ���롣
intermediateDistance = 0.3;
%���ñ���Ŀ��λ���ڼо�����ϵ�¡��ڹ涨�ľ����ڣ�����Ӧλ�ڼг�����z���ϡ�
distanceFromCup.TargetPosition = [0,0,intermediateDistance];%ע���������õĲο�����ϵ��gripper=iiwa_link_ee_kuka������Z����

% ����һ����׼Լ����Ҫ��iiwa_link_ee����ϵ��z����ƴ�ֱ�������ǽ�Ŀ������ڻ������Ϸ���Զ�ĵط�,��Ϊһֱ����úܽ��Ļ���ı���̬��
% iiwa_link_ee����ϵ�Ƕ���ģ�������Լ�����г����뱭����롣
alignWithCup = constraintAiming('iiwa_link_ee');%ע��������ee����ee_kuka������Z����ˮƽ��
alignWithCup.TargetPoint = [0, 0, 100];

%����һ���ؽ�λ�ñ߽�Լ��������ǰ����������ô�Լ����Bounds���ԣ������ƹؽ�λ�õı仯��
limitJointChange = constraintJointBounds(lbr);
%Ϊ�г�������һ������Լ��������Ϊ1�ȡ���Լ��Ҫ��г����ķ�����TargetOrientation����ָ����ֵƥ�䡣
%ʹ�ô�Լ�����̶������ӽ�����ʱ�г����ķ��� ע�������

%����ĩ��ִ��������������ο����ܵķ����ǽ�����ĩ��ִ���������ָ���ķ���ת��Ϊ�����ο�������ָ������ͬ����ķ���
%��ĩ��ִ���������ָ���ķ���ת��Ϊ��ο�������ָ���ķ�����ͬ��
%��Լ��Ҫ��г���gripper�ķ�����TargetOrientation����ָ����ֵƥ�䡣ʹ�ô�Լ�����̶������ӽ�����ʱ�г����ķ���
fixOrientation = constraintOrientationTarget(gripper);
fixOrientation.OrientationTolerance = deg2rad(1);

%Լ��������һ��Ȩֵ���ԣ����������������δ����ͻԼ������Լ����Ȩֵ����Ϊ0�����ø�Լ�������ڴ����ã����ùؽ�λ�ý��޺ͷ���Լ����
limitJointChange.Weights = zeros(size(limitJointChange.Weights));
fixOrientation.Weights = 0;%��Լ����Ȩֵ����Ϊ0�����ø�Լ����Ҳ����2�����û�����Լ���������ڴ����ã����ùؽ�λ�ý��޺ͷ���Լ����

%ʹ��gik����������������Լ���Ļ��������á�����ָ����������Լ����������������Ϊ�ڶ���·�㡣
[qWaypoints(2,:),solutionInfo] = gik(q0, heightAboveTable, ...
                       distanceFromCup, alignWithCup, fixOrientation, ...
                       limitJointChange);

%�ҵ����о���ֱ���ƶ������е����ã��������ùؽ�λ��Լ���ͷ���Լ����
limitJointChange.Weights = ones(size(limitJointChange.Weights));
fixOrientation.Weights = 1;%����Լ����
%���á��뱭���롱Լ������Ϊ����Լ��ʹ�����ࡣ
alignWithCup.Weights = 0; %����Լ����

%����ǰ����������÷���Լ�������ַ���(qWaypoints(2��:))���õ��Ӽг�����������ģ�ͻ����ı任������α任ת��Ϊ��Ԫ����
fixOrientation.TargetOrientation = ...
tform2quat(getTransform(lbr,qWaypoints(2,:),gripper));

%Ϊÿ��·���㶨�屭�Ӻͼо�֮��ľ���
finalDistanceFromCup = 0.05;
distanceFromCupValues = linspace(intermediateDistance, finalDistanceFromCup, numWaypoints-1);

%����ÿ�����е�֮��Ĺؽ�λ����������仯��
maxJointChange = deg2rad(10);

%Ϊÿ��ʣ���·�����������
for k = 3:numWaypoints
    % ����Ŀ��λ�á�
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k-1);
    % ���ؽ�λ������������ǰֵ������
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange, ...
                               qWaypoints(k-1,:)' + maxJointChange];
    % ����������Ⲣ������ӵ�waypoints�����С�
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         heightAboveTable, ...
                                         distanceFromCup, alignWithCup, ...
                                         fixOrientation, limitJointChange);
end

%���ӻ����ɵĹ켣����·����֮�����������ƽ���Ĺ켣��ʹ��pchip������壬���ܻ�Υ�������˵Ĺؽڼ��ޡ�
framerate = 15;
r = rateControl(framerate);
tFinal = 10; %����10S
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];%5��·����ʱ����Ϣ
numFrames = tFinal*framerate; %һ��150�棬�൱�ڲ�150����
%�ֶ����� Hermite ��ֵ����ʽ (PCHIP)
%p = pchip(x,y,xq) ������ xq �еĲ�ѯ���Ӧ�Ĳ�ֵ p ��������p ��ֵ�� x �� y �ı��ηֶ����β�ֵȷ����
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';%150��ؽڽ�

gripperPosition = zeros(numFrames,3); %�о� XYZ
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