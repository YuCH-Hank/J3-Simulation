clc ; clear; close all ;
%% ================== RL Data Check ================== 
% Date  2021/01/29
% =================================
%% ================== Data Processing ================== 
resource = 0; % 0 = mat, 1 = data
if (resource == 0)
    path = 'Data\Command.mat';
    load(path);
    fprintf('Total Usable Command number %d\n', Command.Total_C);
    pick = input('Please Select Command numner : ');
    % pick = 15;
    
else
    Data = load( ['Data\','state_set.txt' ]) ;
    Data = Data(:,1:6) * 180 / pi();

    %         Data_Processing(Input, save_, path,               low_pass)
    Command = Data_Processing(Data, true, 'Data\Command.mat',   false);
    pick = randi(Command.Total_C,1);
    
end

fprintf('Pick Command number %d \n', pick);

Joint = Command.Joint{pick};

%% ================== NURBS Fitting ================== 
Parameter = NURBS_curve_fitting_function (Joint(:,1:6)' , 4  , 3 , 0.001);
JointData = Parameter.Curve;

%% ================== Forward Kinematics ================== 
config = Config('Experiment');
[Object, PLOT, ~, Robot, Record, Home_pose, ~, ~, Scurve_Method] = config.Get_Config;

Controller = Controller('position');
Controller.Collection;

[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

for i = 1 : length(JointData(:,1))
    [ Info  ,  C_NowEulerAngle(i,:) , C_NowPosition(i,:) ] = ForwardKinemetics( DH_table , Robot.pos') ;
    Robot = Controller.Control_Law( [JointData(i, :), JointData(i, :)], Robot, SamplingTime);
    
    Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
    Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
    Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition(i,:)];
    Record.Joint.JointRecord                                            = [Record.Joint.JointRecord;        Robot.pos'];
    Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           JointData(i, :)'];
    Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             zeros(6,1)];
end

%% ================== Plot ================== 
disp('Press Any Botton to Continue......................'); pause;
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

Time = linspace(1,length(JointData(:,1)),length(JointData(:,1))) * SamplingTime;
time_record = length(JointData(:,1));
Record = PlotJointData(Record, Time, time_record, PLOT);