clc; clear; close all;
%% ====================== Robot Simulation Demo ======================
% 2021/02/11
% =======================================================================
global Object Robot Controller Record Scurve_j Scurve_Method TimerFlag EndFlag FinalPose
global ITRI_parameter ITRI_Limitation SamplingTime Home_pose DH_table start

%% >>>> Class
% >>>> Keyboard
kb = HebiKeyboard();

% >>>> Config
config = Config('Experiment');
config.Collection;
[Object, PLOT, Robot, Record, Home_pose, ~, ~, Scurve_Method] = config.Get_Config;

% >>>> Controller
Controller = Class_Controller('position');
Controller.Collection;

% >>>> Scurve
Scurve_j = Scurve_Jerk();

%% >>>> MyRobot
% >>>> ITRI Parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

%% >>>> Parameter
FinalPose = [0, 0, 180];
TimerFlag = 0;
EndFlag = false;
start = false;

%% >>>> Simulatation Process
disp(' ');disp('Press Esc to end program');

while (read(kb).ESC == false)
    
    if (TimerFlag == 0)         %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 0 ');
            Pos = Home_pose';  Reset;
        end
        
        Timer_Stay(Pos, 500);
        
    elseif (TimerFlag == 1)     %% >>>> Path 1 Joint Space PTP
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 1 ');
            FinalPosition = [Object.Center(1), Object.Center(2), Object.Center(3) + Object.Width(3)/2 + Object.distance];
            [Ini, Fin] = Joint_Space(FinalPosition); Reset;
        end
        
        PTP (Ini, Fin);
        
    elseif (TimerFlag == 2)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 2 ');
            Pos = Robot.pos;  Reset;
        end
        
        Timer_Stay(Pos, 200);
        
    elseif (TimerFlag == 3)      %% >>>> Path 2 Cartesian Space PTP
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 3 ');
            [ Info  ,  NowEulerAngle , Ini ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
            FinalPosition = FinalPosition - [0, 0, Object.distance];
            Fin = FinalPosition; Reset;
        end
        
        PTP_C (Ini, Fin);
        
    elseif (TimerFlag == 4)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 4 ');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 200);
        
    elseif (TimerFlag == 5)     %% >>>> Path 3 Cartesian Space PTP to Target Point
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 5 ');
            [ Info  ,  NowEulerAngle , Ini ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
            FinalPosition = FinalPosition + [0, 0, Object.distance*2];
            Fin = FinalPosition; Reset;
        end
        
        PTP_C (Ini, Fin);
        
    elseif (TimerFlag == 6)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 6 ');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 200);
        
    elseif (TimerFlag == 7)     %% >>>> Path 4 Joint Space PTP to Target Point
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 7 ');
            FinalPosition   = [Object.Goal, Object.Width(3) - 27];
            [Ini, Fin] = Joint_Space(FinalPosition); Reset;
        end
        
        PTP (Ini, Fin);
        
    elseif (TimerFlag == 8)     %% >>>> Timer Stay
        if (~start)
            Object.Suction = false; EndFlag = true;
            disp(' ');disp('>>>> TimerFlag == 8 ');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 10);
        
    elseif (TimerFlag == 9)     %% >>>> Path 5 Joint Space PTP to Home Pose
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 9 ');
            Ini = Robot.pos';
            Fin = Home_pose; Reset;
        end
        
        PTP (Ini, Fin);
        
    elseif (TimerFlag == 10)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 10 ');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 200);
        
    elseif (TimerFlag == 11)    %% >>>> End of Program
        break;
        
    end
end

%% >>>> Plot
figure ('name' , 'robot');
for i = 1 : 20 : (length (Record.Time.t) )
    
    Draw_RobotManipulator(  Record.Joint.JointPosRecord( 3*(i-1)+1:3*(i-1)+3 , 1:7 ) , ...
                            Record.Joint.JointDirRecord( 3*(i-1)+1:3*(i-1)+3 , 1:21 ) , ...
                            PLOT.robot.Axis , PLOT.robot.Augmented) ;
    Draw_Trajectory(  Record.Cartesian.EEFRecord(1:i,1:3) );
    Draw_Box(Record.Cartesian.CenterRecord(i,:), Object.Width, Object.Color);
    Draw_Base();
    
    pause(SamplingTime);
    
    hold off;
end

Record = PlotJointData(Record, Record.Time.t, Record.Time.Segment, PLOT);


 %% >>>> Functions
 function [Ini, Fin] = Joint_Space(FinalPosition)
 global Robot Home_pose start FinalPose DH_table
     Ini = Robot.pos';

     % Inverse Kinemetics calculate angle
     [ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( FinalPose , FinalPosition , DH_table ) ;

     % >>>> find best angle
     OptimalSol_Final   = FindOptSol( InverseJointAngle_Final  , Home_pose ) ;
     Fin = OptimalSol_Final;
     start = true;
 end
 
 function Timer_Stay(Pos, time)
 global Robot Controller Object Record TimerFlag SamplingTime DH_table start EndFlag
 
 % >>>> Robot Kinetamics
 [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
 
 % >>>> Send Command (6*1)
 PosCmd = Pos;  VecCmd = zeros(1,6)';
 Robot = Controller.Control_Law( [PosCmd, VecCmd], Robot, SamplingTime);
 
 % >>>> Check Object
 if (~EndFlag)
     if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
         Object.Suction = true;
     end
 end
 
 if (Object.Suction)
     Object.Center = C_NowPosition + [0, 0, -Object.Width(3)/2];
 else
     Object.Center = Object.Center;
 end
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
 Record.Cartesian.CenterRecord                                       = [Record.Cartesian.CenterRecord;   Object.Center];
 Record.Joint.JointRecord                                            = [Record.Joint.JointRecord;        Robot.pos'];
 Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
 Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
 Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           PosCmd];
 Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             VecCmd];
 
% >>>> Time
 if (isnan(Record.Time.t))
     Record.Time.t                                                   = [SamplingTime];
     
 else
     Record.Time.t                                                   = [Record.Time.t,                   Record.Time.t(end) + SamplingTime];

 end
 
 if (isnan(Record.Time.len))
     Record.Time.len                                                 = [SamplingTime];
     
 else
     Record.Time.len                                                 = [Record.Time.len,                 Record.Time.len(end) + SamplingTime];
     
 end
 
 if (length(Record.Time.len) == time)
     Record.Time.Segment                                             = [Record.Time.Segment,             length(Record.Time.len)];
     TimerFlag = TimerFlag + 1;
     Record.Time.len = nan;
     start = false;
     Record.Command.Initial = false;
 end

 
 end
 
 function PTP (Ini, Fin)
 global Robot Controller Object Record TimerFlag SamplingTime DH_table Scurve_Method Scurve_j
 global ITRI_Limitation start EndFlag
 
 % >>>> Initial
 if (~Record.Command.Initial)
     if (Scurve_Method.All)
         Option = Scurve_Method.PathAll;
     else
         Option = Scurve_Method.P1;
     end
     
     switch Option  % 0: original, 1: time sync, 2: Jerk
         case 0
             % >>>> parameter
             acc_avg = 0.75;
             [ JointCmd , Time1 ] = Scurve_MultiAxis ( Ini , Fin , SamplingTime , acc_avg , ITRI_Limitation);
         case 1
             Scurve_j.Clear;
             Scurve_j.Input(   Ini, Fin, ...
                 ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
                 ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
                 sqrt(3)/2, SamplingTime);
             Scurve_j.MultiAxis_Time_Sync;
             [ JointCmd , Time1 ] = Scurve_j.Get_Command;
         case 2
             Scurve_j.Clear;
             Scurve_j.Input(   Ini, Fin, ...
                 ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
                 ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
                 sqrt(3)/2, SamplingTime);
             Scurve_j.MultiAxis_Time_optimial_Cmd;
             [ JointCmd , Time1 ] = Scurve_j.Get_Command;
     end
     
     Record.Command.Initial      = true;
     Record.Command.Position     = JointCmd(1:6,:);
     Record.Command.Velocity     = JointCmd(7:12,:);
     Record.Command.length       = length(Time1);
     Record.Command.count        = 1;
 end
 
 % >>>> Robot Kinetamics
 [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
 
 % >>>> Send Command (6*1)
 PosCmd = Record.Command.Position(:,Record.Command.count);  VecCmd = Record.Command.Velocity(:,Record.Command.count);
 Robot = Controller.Control_Law( [PosCmd, VecCmd], Robot, SamplingTime);
 
 % >>>> Check Object
 if (~EndFlag)
     if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
          Object.Suction = true;
     end
 end
 
 if (Object.Suction)
     Object.Center = C_NowPosition + [0, 0, -Object.Width(3)/2];
 else
     Object.Center = Object.Center;
 end
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
 Record.Cartesian.CenterRecord                                       = [Record.Cartesian.CenterRecord;   Object.Center];
 Record.Joint.JointRecord                                            = [Record.Joint.JointRecord;        Robot.pos'];
 Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
 Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
 Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           PosCmd];
 Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             VecCmd];
 Record.Command.count                                                = Record.Command.count + 1;
 
 % >>>> Time
 if (isnan(Record.Time.t))
     Record.Time.t                                                   = [SamplingTime];
     
 else
     Record.Time.t                                                   = [Record.Time.t,                   Record.Time.t(end) + SamplingTime];

 end
 
 if (isnan(Record.Time.len))
     Record.Time.len                                                 = [SamplingTime];
     
 else
     Record.Time.len                                                 = [Record.Time.len,                 Record.Time.len(end) + SamplingTime];
     
 end
 
 if (length(Record.Time.len) == Record.Command.length)
     Record.Time.Segment                                             = [Record.Time.Segment,             length(Record.Time.len)];
     TimerFlag = TimerFlag + 1;
     Record.Time.len = nan;
     start = false;
     Record.Command.Initial = false;
 end
 end
 
 function PTP_C (Ini, Fin)
 global Robot Controller Object Record TimerFlag SamplingTime DH_table Scurve_Method Scurve_j start FinalPose
 
 % >>>> Initial
 if (~Record.Command.Initial)
     if (Scurve_Method.All)
         Option = Scurve_Method.PathAll;
     else
         Option = Scurve_Method.P1;
     end
     
     switch Option  % 0: original, 1: time sync, 2: Jerk
         case 0
             % >>>> parameter
             acc_lim = 100;       % (m/s^2)
             vec_lim = 50;       % (m/s)
             acc_avg = 0.75;
             
             [ Cartesian_Cmd , Time ] = Scurve ( Ini , Fin ,  acc_lim , acc_avg , vec_lim , SamplingTime);
         case 1
             % >>>> parameter
             acc_lim = [100,100,100];       % (m/s^2)
             vec_lim = [50,50,50];       % (m/s)
             Scurve_j.Clear;
             Scurve_j.Input(   Ini, Fin, ...
                 vec_lim,  acc_lim, ...
                 acc_lim * 10, acc_lim * 100, ...
                 sqrt(3)/2, SamplingTime);
             Scurve_j.MultiAxis_Time_Sync;
             [ Cartesian_Cmd , Time ] = Scurve_j.Get_Command;
         case 2
             % >>>> parameter
             acc_lim = [100,100,100];       % (m/s^2)
             vec_lim = [50,50,50];       % (m/s)
             Scurve_j.Clear;
             Scurve_j.Input(   Ini, Fin, ...
                 vec_lim,  acc_lim, ...
                 acc_lim * 10, acc_lim * 100, ...
                 sqrt(3)/2, SamplingTime);
             Scurve_j.MultiAxis_Time_optimial_Cmd;
             [ Cartesian_Cmd , Time ] = Scurve_j.Get_Command;
     end
     
     Record.Command.Initial      = true;
     Record.Command.Position     = Cartesian_Cmd(1:3,:);
     Record.Command.Velocity     = Cartesian_Cmd(4:6,:);
     Record.Command.length       = length(Time);
     Record.Command.count        = 1;
 end
 
 
 % >>>> Robot Kinetamics
 [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
 
 % Inverse Kinemetics calculate angle
 [ InverseAngle , SingularFlag2 ] = InverseKinemetics( FinalPose , Record.Command.Position(1:3, Record.Command.count)' , DH_table ) ;
 OptimalSol   = FindOptSol( InverseAngle , Robot.pos' ) ;
 
 % >>>> 基於Robot Jacobian軌跡
 [ RobotJacobian ] = ComputeRobotJacobian( DH_table , Robot.pos' );
 JointVelocity =  inv( RobotJacobian )  * ...
     [ Record.Command.Velocity(1, Record.Command.count) ; Record.Command.Velocity(2, Record.Command.count) ; Record.Command.Velocity(3, Record.Command.count) ; 0 ; 0 ; 0 ] ;
 
 % >>>> Send Command (6*1)
 PosCmd = OptimalSol';  VecCmd = JointVelocity;
 Robot = Controller.Control_Law( [PosCmd, VecCmd], Robot, SamplingTime);
 
 % >>>> Check Object
 if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
      Object.Suction = true;
 end
 
 if (Object.Suction)
     Object.Center = C_NowPosition + [0, 0, -Object.Width(3)/2];
 else
     Object.Center = Object.Center;
 end
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
 Record.Cartesian.CenterRecord                                       = [Record.Cartesian.CenterRecord;   Object.Center];
 Record.Joint.JointRecord                                            = [Record.Joint.JointRecord;        Robot.pos'];
 Record.Joint.JointPosRecord                                         = [Record.Joint.JointPosRecord;     Info.JointPos];
 Record.Joint.JointDirRecord                                         = [Record.Joint.JointDirRecord;     Info.JointDir];
 Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           PosCmd];
 Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             VecCmd];
 Record.Command.count                                                = Record.Command.count + 1;
 
 % >>>> Time
 if (isnan(Record.Time.t))
     Record.Time.t                                                   = [SamplingTime];
     
 else
     Record.Time.t                                                   = [Record.Time.t,                   Record.Time.t(end) + SamplingTime];
     
 end
 
 if (isnan(Record.Time.len))
     Record.Time.len                                                 = [SamplingTime];
     
 else
     Record.Time.len                                                 = [Record.Time.len,                 Record.Time.len(end) + SamplingTime];
     
 end
 
 if (length(Record.Time.len) == Record.Command.length)
     Record.Time.Segment                                             = [Record.Time.Segment,             length(Record.Time.len)];
     TimerFlag = TimerFlag + 1;
     Record.Time.len = nan;
     start = false;
     Record.Command.Initial = false;
 end
 end
 
 function Reset
 global Controller start
 
 Controller.velocity.error = [];
 start = true;
 end