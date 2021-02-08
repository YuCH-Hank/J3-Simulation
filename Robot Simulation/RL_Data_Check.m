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

% >>>> Scurve class
Scurve_j = Scurve_Jerk();

%% ================== NURBS Fitting ================== 
Parameter = NURBS_curve_fitting_function (Joint(:,1:6)' , 4  , 3 , 0.001);
JointData = Parameter.Curve;

%% ================== Configuration ================== 
% >>>> ITRI_parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

config = Config('Experiment');
[Object, PLOT, ~, Robot, Record, Home_pose, ~, ~, Scurve_Method] = config.Get_Config;

Controller = Class_Controller('position');
Controller.Collection;

%% Path Planning
% >>>> Joint space Scurve

if (Scurve_Method.All)
    Option = Scurve_Method.PathAll;
else
    Option = Scurve_Method.P1;
end

switch Option  % 0: original, 1: time sync, 2: Jerk
    case 0
        % >>>> parameter
        acc_avg = 0.75;
        [ JointCmd , Time1 ] = Scurve_MultiAxis ( Home_pose , OptimalSol_Final , SamplingTime , acc_avg , ITRI_Limitation);
    case 1
        Scurve_j.Input(   Home_pose, OptimalSol_Final, ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_Sync;
        [ JointCmd , Time1 ] = Scurve_j.Get_Command;
    case 2
        Scurve_j.Input(   Home_pose, JointData(1,1:6)', ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_optimial_Cmd;
        [ JointCmd , Time1 ] = Scurve_j.Get_Command;
end


%% Simulation Process
% >>>> Get Initial Angle
NowJoint = Home_pose;

% >>>> Joint space Scurve Simulation process
for i = 1 : length( Time1 )

    % >>>> Robot Kinetamics
    [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % >>>> Controller
    Robot = Controller.Control_Law( [JointCmd(1:6,i),JointCmd(7:12,i)], Robot, SamplingTime);
    
    % >>>> Check Object
    if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
        Object = true;
    end
    
    if (Object.Suction)
        Object.Center = C_NowPosition + [0, 0, -Object.Width(3)/2];
    else
        Object.Center = Object.Center;
    end
   
    % >>>> Record file
    Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
    Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];   
    Record.Cartesian.CenterRecord                                       = [Record.Cartesian.CenterRecord;   Object.Center];    
    Record.Joint.JointRecord                                            = [Record.Joint.JointRecord;        Robot.pos'];
    Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
    Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
    Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           JointCmd(1:6,i)];
    Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             JointCmd(7:12,i)];
   
end

%% ================== Forward Kinematics ================== 

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

disp('Press Any Botton to Continue......................'); pause;
figure('name' , 'Cartesian');
plot3(C_NowPosition(:,1),C_NowPosition(:,2),C_NowPosition(:,3))
grid on;
xlabel('x'); ylabel('y'); zlabel('z');

Time = [Time1, linspace(1,length(JointData(:,1)),length(JointData(:,1))) * SamplingTime + Time1(end)];
time_record = [length(Time1); length(JointData(:,1))];
Record = PlotJointData(Record, Time, time_record, PLOT);