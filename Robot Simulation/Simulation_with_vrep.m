clear, clc, close all;
%% Vrep with MatLab Simulation
% Date: 2021/01/21
% =================================================================================================
%%  ==================================== Parameter =========================================

[Object, PLOT, Controller, Robot, Record, Home_pose] = Config('Experiment');

%% ==================================== Vrep ============================================= 

M_vrep = VrepSimulation();

M_vrep.Initial_vrep;        % >>>> Initial

M_vrep.Get_handle;          % >>>> Get handle

M_vrep.Start_sync;          % >>>> Start Sync

M_vrep.Sent_JointPosCmd_Quick(Home_pose); % >>>> Set Home pose

%% ==================================== J3 System Model ==================================== 

% >>>> ITRI_parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint Scurve 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% >>>> final pose
FinalPose       = [0, 0, 180];
FinalPosition   = [ M_vrep.Cubid.pos(1) * 100, ...
                    M_vrep.Cubid.pos(2) * 100, ...
                    M_vrep.Cubid.pos(3) * 100 + M_vrep.Cubid.height/2+ M_vrep.Cubid.distance];

% Inverse Kinemetics calculate angle
[ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( FinalPose , FinalPosition , DH_table ) ;

% >>>> find best angle
OptimalSol_Final = FindOptSol( InverseJointAngle_Final , Home_pose ) ;

%% >>>> Path Planning        >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% >>>> Joint space Scurve
[ JointCmd , Time1 ] = Scurve_MultiAxis ( Home_pose , OptimalSol_Final , SamplingTime , 0.75 , ITRI_Limitation);

% >>>> Get Initial Angle
NowJoint = Home_pose;

%% >>>> Simulation Process   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

for i = 1 : length( Time1 )

    % >>>> Robot >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    % >>>> Robot Kinetamics
    [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % >>>> Robot Controller   
    [Robot, Controller] = Control(Robot, Controller, [JointCmd(1:6,i), JointCmd(7:12,i)], SamplingTime);
    
    % >>>> M Vrep >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    M_vrep.Sent_JointPosCmd(JointCmd(1:6,i));       % >>>> Sent PosCmd
    
    M_vrep.Get_JointPos;                            % >>>> Get JointCmd
    
    M_vrep.Get_CubidPos;                            % >>>> Get Cubid Pos;
    
    M_vrep.Trigger;                                 % >>>> Trigger
      
    % >>>> Record file >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    Record.Cartesian.EEFRecord                      = [Record.Cartesian.EEFRecord;      C_NowPosition];
    Record.Cartesian.EulerAngle                     = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];   
    Record.Cartesian.CenterRecord                   = [Record.Cartesian.CenterRecord;   M_vrep.Cubid.pos' .*100];    
    Record.Joint.JointRecord                        = [Record.Joint.JointRecord;        Robot.pos'];
    Record.Joint.JointPosRecord                     = [Record.Joint.JointPosRecord;     Info.JointPos];
    Record.Joint.JointDirRecord                     = [Record.Joint.JointDirRecord;     Info.JointDir];
    Record.Joint.JointCmd                           = [Record.Joint.JointCmd,           JointCmd(1:6,i)];
    Record.Joint.VelCmd                             = [Record.Joint.VelCmd,             JointCmd(7:12,i)];
    Record.vrep.joint                               = [Record.vrep.joint,               M_vrep.Joint.pos'];
   
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Catesian Scurve 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% >>>> get now pose
[ Info  ,  NowEulerAngle , InitialPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
FinalPosition = FinalPosition - [0, 0, M_vrep.Cubid.distance];

%% >>>> Path Planning        >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% >>>> parameter 
acc_lim = 100;       % (m/s^2)
vec_lim = 50;       % (m/s)
acc_avg = 0.75;

% >>>> Catesian space Scurve
[ Cartesian_Cmd , Time2 ] = Scurve ( InitialPosition , FinalPosition ,  acc_lim , acc_avg , vec_lim , SamplingTime);
Controller.velocity.error = [];

%%  Simulation Process

% >>>> Catesian space Simulation process
for i = 1 : length( Time2 )
    
    % >>>> Robot >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    % >>>> Robot Kinetamics
    [ Info ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % >>>> Inverse Kinemetics calculate angle
    [ InverseAngle , SingularFlag2 ] = InverseKinemetics( FinalPose , Cartesian_Cmd(1:3,i)' , DH_table ) ;
    OptimalSol   = FindOptSol( InverseAngle , Robot.pos' ) ;
    
    % >>>> 基於Robot Jacobian軌跡
    [ RobotJacobian ] = ComputeRobotJacobian( DH_table , Robot.pos' );
    JointVelocity =  inv( RobotJacobian )  * [ Cartesian_Cmd(4,i) ; Cartesian_Cmd(5,i) ; Cartesian_Cmd(6,i) ; 0 ; 0 ; 0 ] ; 
    
    % >>>> Robot Controller
    [Robot, Controller] = Control(Robot, Controller, [OptimalSol', JointVelocity], SamplingTime);
    
    % >>>> M Vrep >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    % >>>> Sent PosCmd
    M_vrep.Sent_JointPosCmd(OptimalSol');
    
    % >>>> Get JointCmd
    M_vrep.Get_JointPos;
    
    % >>>> Trigger
    M_vrep.Trigger;
    
    % >>>> Record file >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    Record.Cartesian.EEFRecord                      = [Record.Cartesian.EEFRecord;      C_NowPosition];
    Record.Cartesian.EulerAngle                     = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];   
    Record.Cartesian.CenterRecord                   = [Record.Cartesian.CenterRecord;   M_vrep.Cubid.pos' .*100];    
    Record.Joint.JointRecord                        = [Record.Joint.JointRecord;        Robot.pos'];
    Record.Joint.JointPosRecord                     = [Record.Joint.JointPosRecord;     Info.JointPos];
    Record.Joint.JointDirRecord                     = [Record.Joint.JointDirRecord;     Info.JointDir];
    Record.Joint.JointCmd                           = [Record.Joint.JointCmd,           OptimalSol'];
    Record.Joint.VelCmd                             = [Record.Joint.VelCmd,             JointVelocity];
    Record.vrep.joint                               = [Record.vrep.joint,               M_vrep.Joint.pos'];
    
end
%% ==================================== Plot  ==================================== 
%% Compute Error & Plot

if (PLOT.robot.visible)
    % >>>> Show Trajectory
    figure('name','ShowTrajectory');

    switch PLOT.path
        case 1
            len = length( Time1 );
        case 2
            len = length( Time1 ) + length(Time2) ;
        case 3
            len = length( Time1 ) + length(Time2) + length(Time3);
        case 4
            len = length( Time1 ) + length(Time2) + length(Time3) + length(Time4) ;
        case 5
            len = length( Time1 ) + length(Time2) + length(Time3) + length(Time4) + length(Time5);
    end
    for i = 1 : 20 : len

         Draw_RobotManipulator( Record.Joint.JointPosRecord( 3*(i-1)+1:3*(i-1)+3 , 1:length(Info.JointPos) ) , ...
                                Record.Joint.JointDirRecord( 3*(i-1)+1:3*(i-1)+3 , 1:length(Info.JointDir) ) , ...
                                PLOT.robot.Axis , PLOT.robot.Augmented) ;
         Draw_Trajectory(  Record.Cartesian.EEFRecord(1:i,1:3) );
         Draw_Base();
         Draw_Box(Record.Cartesian.CenterRecord(i,:), Object.Width, Object.Color);
         pause(SamplingTime);

         hold off;
    end
end
%%
figure('name', 'cmd');
for i = 1:6
   subplot(2,3,i)
   plot([Time1, Time2 + Time1(end)],  Record.vrep.joint(i,:),'r--',...
        [Time1, Time2 + Time1(end)],  Record.Joint.JointRecord(:,i),'b--',...
        [Time1, Time2 + Time1(end)],  Record.Joint.JointCmd(i,:),'k--');
    legend('vrep', 'robot', 'cmd');
end

%% ==================================== Stop Simulation ==================================== 
M_vrep.StopSimulation;