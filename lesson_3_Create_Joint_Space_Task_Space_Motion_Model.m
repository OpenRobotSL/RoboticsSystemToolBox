%Create Joint-Space Motion Model
clear all
robot = loadrobot("kinovaGen3","DataFormat","column","Gravity",[0 0 -9.81]);
tspan = 0:0.01:2;
initialState = [homeConfiguration(robot); zeros(7,1)];
targetState = [pi/4; pi/3; pi/2; -pi/3; pi/4; -pi/4; 3*pi/4; zeros(7,1); zeros(7,1)];
motionModel = jointSpaceMotionModel("RigidBodyTree",robot);

%ע�����涼�Ǳջ�����ѧ����
motionModel.NaturalFrequency %����ѧ����Ƶ�ʣ�������"ComputedTorqueControl" or "IndependentJointMotion".����Ч
motionModel.DampingRatio %��������ѧ������ȣ�ָ��Ϊʵֵ�ı�����nԪ������
motionModel.Kp %��PDģ�Ϳ�������Ч
motionModel.Kd

% �˶����ͣ�ָ��Ϊ�ַ����������ַ��������������ģ�ı�ѭ�����Ͽռ���Ϊ��ѡ����:
% ��ComputedTorqueControl��-����ȫ����ѧ������������ȻƵ�ʺ�DampingRatio������ָ��������ѧ����Ĭ��ģʽ������Ҫ�ο�λ���ٶȼ��ٶȣ�
% �������ؽ��˶�������ʹ����ȻƵ�ʺ���������ָ��������ѧ����ÿ���ؽڽ�ģΪһ�������Ķ���ϵͳ������Ҫ�ο�λ���ٶȼ��ٶȣ�
% ��PDControl��-����ָ����Kp��Kd���ԶԹؽڽ��б���΢�ֿ��ơ�����Ӧ�þ��ǵ��PD���ƣ�������PD���ؿ��Ʋ�ͬ��ֻ��Ҫ�ο�λ���ٶȣ�

%ͨ������ת�ؿ��ƺ���5%�������еȿ��ٽ�Ծ��Ӧ���������ѧ����ϵͳ���н�ģ��
updateErrorDynamicsFromStep(motionModel,.3,.05);%���ó�0�Ͳ���������������Ƕ���ѧת�ؿ���

[t,robotState] = ode45(@(t,state) derivative(motionModel,state,targetState),tspan,initialState);
%@(t,state)derivative, t state��Ϊodefun-der()���������,state��current state��ǰ������״̬
%��������ʹ��https://ww2.mathworks.cn/help/matlab/math/parameterizing-functions.html
%@(t,state) derivative(motionModel,state,targetState)
%��˼��motionModel,targetState��derivative���������������������������������õ���der�����Լ��йصĲ���������ode�������α���
%ode���� https://ww2.mathworks.cn/help/matlab/ref/ode15s.html#bu5yps7_sep_shared-odefun
%ode(@(t,state))�ⲿ����ode�������Σ�ͬʱder������Ҳ����state�������������ܼ���΢��

figure
plot(t,robotState(:,1:motionModel.NumJoints)); %1-7�йؽ�λ��
hold all;
plot(t,targetState(1:motionModel.NumJoints)*ones(1,length(t)),"--"); %�ο�Ŀ��ؽ�λ��
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
%�˶����ͣ�ָ��Ϊ��PDControl������ʹ�ñ�������(PD)����(�ѿ����ռ�)ӳ�䵽�ؽ�ͨ��һ���ſ˱�ת�ÿ��������ÿؼ�����ָ����Kp��Kd���ԡ�
motionModel.MotionType
%ʹ��Ӳ��������1����ģ����Ϊ���Ը���Ч�ز�������˶�̬��ʹ��ode15s�����ڸ߱仯�ʵ�������Χʵ�ָ��ߵľ��ȡ�
%ע���������ٶȲ���������ĩ�ٶ�0��
[t,robotState] = ode15s(@(t,state)derivative(motionModel,state,refPose,refVel),tspan,initialState);
%ode15s������΢�ַ��̺� DAE - ��׷�����
%derivative��е��ģ��״̬��ʱ�䵼��
%���� MATLAB? ODE ����������Խ��� y��=f(t,y) ��ʽ�ķ����飬���漰�������� M(t,y)y��=f(t,y) �����⡣
%�������ʹ�����Ƶ��﷨��ode23s �����ֻ�ܽ�����������Ϊ���������⡣
%ode15s �� ode23t ���Խ����������������������⣬��Ϊ΢�ִ������� (DAE)��ʹ�� odeset �� Mass ѡ��ָ����������
%state�ؽ�λ�ú��ٶȱ�ʾΪһ��1*2nԪ��������ʾΪ[q;qDot]��n��motionModel�Ĺ����������еĹؽ�������ʾÿ���ؽڵ�λ�ã���λΪ���ȡ�qDot��ʾÿ���ؽڵ��ٶȣ���λΪ����/�롣
figure
show(robot,initialState(1:7));
hold all
plot3(refPose(1,4),refPose(2,4),refPose(3,4),"x","MarkerSize",20)

r = rateControl(100);%�۲��������5hz��·�е���Ӧ��
for i = 1:size(robotState,1)
    show(robot,robotState(i,1:7)',"PreservePlot",false);
    waitfor(r);  %��ͣ����ִ���ԴﵽԤ�ڵ�ִ���ٶ�
end

%%%%%%%%%%%%
%ע��derivative ���ص��ٶȣ�Ҳ����˵ode��ͨ��dy/dt=[]΢�ַ��̼���y������der����dy/dt��













