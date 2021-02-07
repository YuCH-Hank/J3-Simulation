clc ; clear; close all ;
%% RL Data Check
% Date  2021/01/29
% =================================
%% ================== Data Processing ================== 
resource = 1; % 0 = mat, 1 = data
if (resource == 0)
    path = 'Data\Command.mat';
    load(path);
    fprintf('Total Usable Command number %d\n', Command.Total_C);
else
    Data = load( ['Data\','state_set.txt' ]) ;
    Data = Data(:,1:6) * 180 / pi();

    %         Data_Processing(Input, save_, path,               low_pass)
    Command = Data_Processing(Data, true, 'Data\Command.mat',   false);
end

pick = randi(Command.Total_C,1);
fprintf('Pick Command number %d \n', pick);

Joint = Command.Joint{pick};

%% ================== NURBS Fitting ================== 
Parameter = NURBS_curve_fitting_function (Joint(:,1:6)' , 5  , 3 , 0.001);
JointData = Parameter.Curve;

%% ================== Forward Kinematics ================== 
config = Config('Experiment');
[Object, PLOT, Controller, Robot, Record, Home_pose, ~, ~, Scurve_Method] = config.Get_Config;

[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

for i = 1 : length(JointData(:,1))
    [ Info  ,  C_NowEulerAngle(i,:) , C_NowPosition(i,:) ] = ForwardKinemetics( DH_table , JointData(i,:) ) ;
    Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
    Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
    Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition(i,:)];
end

%% ================== Plot ================== 
figure ('name' , 'robot');
for i = 1 : 20 : length(JointData(:,1))
    
    Draw_RobotManipulator(  Record.Joint.JointPosRecord( 3*(i-1)+1:3*(i-1)+3 , 1:length(Info.JointPos) ) , ...
                            Record.Joint.JointDirRecord( 3*(i-1)+1:3*(i-1)+3 , 1:length(Info.JointDir) ) , ...
                            PLOT.robot.Axis , PLOT.robot.Augmented) ;
    Draw_Trajectory(  Record.Cartesian.EEFRecord(1:i,1:3) );
    Draw_Base();
    pause(SamplingTime);
    
    hold off;
end

figure('name' , 'Cartesian');
plot3(C_NowPosition(:,1),C_NowPosition(:,2),C_NowPosition(:,3))
grid on;
xlabel('x'); ylabel('y'); zlabel('z');