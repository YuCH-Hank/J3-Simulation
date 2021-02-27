clc; clear; close all;
%% ====================== Robot Simulation Demo ======================
% 2021/02/21
% =======================================================================
global Robot Controller Record Scurve_j Scurve_Method TimerFlag EndFlag 
global ITRI_parameter ITRI_Limitation SamplingTime Home_pose DH_table start

%% >>>> Class
% >>>> Keyboard
kb = HebiKeyboard();

% >>>> Config
config = Config('Experiment');
config.Collection;
[~, PLOT, Robot, Record, Home_pose, ~, ~, Scurve_Method] = config.Get_Config;

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
TimerFlag = 0;
EndFlag = false;
start = false;
count = 1;

%% >>>> Get File
ch = 6;
switch ch
    case 0
        path = 'Data/Scurve_Minimize_Jerk.txt'
    case 1
        path = 'Data/Scurve_MultiAxisLimitation.txt'
    case 2
        path = 'Data/Scurve_Original.txt'
    case 3
        path = 'Data/Scurve_TimeSync.txt'
    case 4
        path = 'Data/C_Minimum_Jerk.txt'
    case 5
        path = 'Data/C_Scurve_MultiAxis.txt'
    case 6
        path = 'Data/C_Time_sync.txt'
end
Command  = load(path);

%% >>>> Simulatation Process
disp(' ');disp('Press Esc to end program');

while (read(kb).ESC == false)
    
    if (TimerFlag == 0)         %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 0 - TimerStay');
            Pos = Home_pose';  Reset;
        end
        
        Timer_Stay(Pos, 100);
        
    elseif (TimerFlag == 1)     %% >>>> Path 1 PTP to InitialPosition
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 1 - PTP to InitialPosition');
            Ini =  Home_pose ; Fin = Command(1,1:6);
            Reset;
        end
        
        PTP (Ini, Fin);
        
    elseif (TimerFlag == 2)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 2 - TimerStay');
            Pos = Robot.pos;  Reset;
        end
        
        Timer_Stay(Pos, 100);
        
    elseif (TimerFlag == 3)      %% >>>> Path 2 Tracking
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 3 - Tracking');Reset;
        end
        
        Track (Command(count, 1:6)', Command(count, 7:12)', length(Command));
        count = count + 1;
        
    elseif (TimerFlag == 4)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 4 - TimerStay');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 100);
        
    elseif (TimerFlag == 5)     %% >>>> Path 3 PTP to Home Position
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 5 - PTP to Home Position');
            Ini = Robot.pos'; Fin = Home_pose; Reset;
        end
        
        PTP (Ini, Fin);
        
    elseif (TimerFlag == 6)     %% >>>> Timer Stay
        if (~start)
            disp(' ');disp('>>>> TimerFlag == 6 - TimerStay');
            Pos = Robot.pos; Reset;
        end
        
        Timer_Stay(Pos, 100);
        
    elseif (TimerFlag == 7)     %% >>>> End of Program
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
    Draw_Base();
    
    pause(SamplingTime);
    
    hold off;
end

Record = PlotJointData(Record, Record.Time.t, Record.Time.Segment, PLOT);


 %% >>>> Functions
 function Timer_Stay(Pos, time)
 global Robot Controller Record TimerFlag SamplingTime DH_table start 
 
 % >>>> Robot Kinetamics
 [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
 
 % >>>> Send Command (6*1)
 PosCmd = Pos;  VecCmd = zeros(1,6)';
 Robot = Controller.Control_Law( [PosCmd, VecCmd], Robot, SamplingTime);
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
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
     fprintf('\t >>>> %2fs\n', time * SamplingTime);
 end

 end
 
 function Track(Pos, Vec, len)
  global Robot Controller Record TimerFlag SamplingTime DH_table start 
 
 % >>>> Robot Kinetamics
 [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
 
 % >>>> Send Command (6*1)
 PosCmd = Pos;  VecCmd = Vec;
 Robot = Controller.Control_Law( [PosCmd, VecCmd], Robot, SamplingTime);
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
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
 
 if (length(Record.Time.len) == len)
     Record.Time.Segment                                             = [Record.Time.Segment,             length(Record.Time.len)];
     TimerFlag = TimerFlag + 1;
     Record.Time.len = nan;
     start = false;
     Record.Command.Initial = false;
     fprintf('\t >>>> %2fs\n', len * SamplingTime);
 end
 end
 
 function PTP (Ini, Fin)
 global Robot Controller Record TimerFlag SamplingTime DH_table Scurve_Method Scurve_j
 global ITRI_Limitation start
 
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
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
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
     fprintf('\t >>>> %2fs\n', Record.Command.length * SamplingTime);
 end
 end
 
 function PTP_ (Ini, Fin)
 global Robot Controller Record TimerFlag SamplingTime DH_table Scurve_Method Scurve_j
 global ITRI_Limitation start
 
 % >>>> Initial
 if (~Record.Command.Initial)
     if (Scurve_Method.All)
         Option = Scurve_Method.PathAll;
     else
         Option = Scurve_Method.P1;
     end
     Option = 0;
     
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
 
 % >>>> Record
 Record.Cartesian.EEFRecord                                          = [Record.Cartesian.EEFRecord;      C_NowPosition];
 Record.Cartesian.EulerAngle                                         = [Record.Cartesian.EulerAngle;     C_NowEulerAngle];
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
     fprintf('\t >>>> %2fs\n', Record.Command.length * SamplingTime);
 end
 end
 
 function Reset
 global Controller start
 
 Controller.velocity.error = [];
 start = true;
 end