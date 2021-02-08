clc ; clear; close all ;
%% Simulation for J3 Nozzle pick and place
% Date  2001/01/17
% =================================
%% >>>> J3 System Model

% >>>> ITRI_parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

% >>>> Scurve class
Scurve_j = Scurve_Jerk();

%% >>>> Stuff
config = Config('RL');
config.Collection;
[Object, PLOT, Controller, Robot, Record, Home_pose, RL, Attractive, Scurve_Method] = config.Get_Config;
                    
% >>>> Delay
delay = [100, 100, 100, 0];
t1 = 0;

if (PLOT.path > 0)
%% >>>> Forward Kinematics 

% >>>> final pose
FinalPose       = [0, 0, 180];
FinalPosition   = [Object.Center(1), Object.Center(2), Object.Center(3) + Object.Width(3)/2 + Object.distance];

% Inverse Kinemetics calculate angle
[ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( FinalPose , FinalPosition , DH_table ) ;

% >>>> find best angle
OptimalSol_Final   = FindOptSol( InverseJointAngle_Final   , Home_pose ) ;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint Scurve 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Path Planning

[ JointCmd_Seg, Endflag] = PosCmd_segment(Home_pose, OptimalSol_Final, RL.max_step_number, RL.segment_size);
% >>>> Get Initial Angle
NowJoint = Home_pose;

fprintf('\n>>>>>>>>>>>>>>>> Start Simulation Joint Scurve 1 ...............\n');
if(~Endflag)
    % >>>> Joint space Scurve
    for i = 2 : length(JointCmd_Seg(:,1))
        count = 1;
        if (Scurve_Method.All)
            Option = Scurve_Method.PathAll;
        else
            Option = Scurve_Method.P1;
        end
        
        switch Option  % 0: original, 1: time sync, 2: Jerk
            case 0
                % >>>> parameter
                acc_avg = 0.75;
                [ JointCmd , time ] = Scurve_MultiAxis (  Robot.pos' , JointCmd_Seg(i,:) , SamplingTime , acc_avg , ITRI_Limitation);
            case 1
                Scurve_j.Clear;
                Scurve_j.Input(    Robot.pos' , JointCmd_Seg(i,:), ...
                    ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
                    ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
                    sqrt(3)/2, SamplingTime);
                Scurve_j.MultiAxis_Time_Sync;
                [ JointCmd , time ] = Scurve_j.Get_Command;
            case 2
                Scurve_j.Clear;
                Scurve_j.Input(    Robot.pos' , JointCmd_Seg(i,:), ...
                    ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
                    ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
                    sqrt(3)/2, SamplingTime);
                Scurve_j.MultiAxis_Time_optimial_Cmd;
                [ JointCmd , time ] = Scurve_j.Get_Command;
        end
        
        
        %% Simulation Process
        fprintf('>>>> J_Seg\t %d/%d \tSteps = %d \t time = %.2fs\n',i, length(JointCmd_Seg(:,1)), length(JointCmd(1,:)), t1 * SamplingTime);

        while (sqrt(sum(  (JointCmd_Seg(i,:) - Robot.pos').^2 )) > RL.thereshould)
            
            % >>>> Robot Kinetamics
            [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
            
            % >>>> Controller
            switch Controller.type
                case 'position'
                    % >>>> Joint space Scurve
                    [Robot, Controller] = Control(Robot, Controller, [JointCmd(1:6, count), JointCmd(1:6, count)], SamplingTime);
                    Vattractive = zeros(6,1);
                    
                case 'velocity'
                    %                     Vattractive = (Attractive.Vmin + ( Attractive.Vmax - Attractive.Vmin ) ).*...
                    %                         (sqrt(sum(  (JointCmd(i,:)' - Robot.pos).^2 ))) .* (sign(JointCmd(i,:)' - Robot.pos));
                    
                    distance = [1 / ( 1 + exp( -1 * (sqrt((JointCmd(1,i)' - Robot.pos(1))^2))));
                                1 / ( 1 + exp( -1 * (sqrt((JointCmd(2,i)' - Robot.pos(2))^2))));
                                1 / ( 1 + exp( -1 * (sqrt((JointCmd(3,i)' - Robot.pos(3))^2))));
                                1 / ( 1 + exp( -1 * (sqrt((JointCmd(4,i)' - Robot.pos(4))^2))));
                                1 / ( 1 + exp( -1 * (sqrt((JointCmd(5,i)' - Robot.pos(5))^2))));
                                1 / ( 1 + exp( -1 * (sqrt((JointCmd(6,i)' - Robot.pos(6))^2))))];
                    
                    Vattractive = (Attractive.Vmin + ( Attractive.Vmax - Attractive.Vmin ) .* distance  )...
                                .* ( sign(JointCmd(1:6, count) - Robot.pos) );
            end
            
            % >>>> Controller
            [Robot, Controller] = Control(Robot, Controller, [JointCmd(1:6,count), Vattractive], SamplingTime);
                        
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
            Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           JointCmd(1:6,count)];
            Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             JointCmd(7:12,count)];
            count = count + 1;
            t1 = t1 + 1;
            time = time + SamplingTime;
            
            if (count > length(JointCmd(1,:)))
                fprintf('>>>>>>>>>>>>>>>>>>..... exceed\n')
                break;
            end
        end
        
        if (RL.min)
            if (time < RL.min_time)
                [Robot, Record] = Robot_delay(Robot, Object, round((RL.min_time - time)/SamplingTime), Controller, SamplingTime, Record, DH_table);
                t1 = t1 + round((RL.min_time - time)/SamplingTime);
            end
            
            time = 0;
        end
    end
end
Time1 = linspace(1, t1 - 1, t1) .* SamplingTime;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Catesian Scurve 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (PLOT.path > 1)
[Robot, Record] = Robot_delay(Robot, Object, delay(1), Controller, SamplingTime, Record, DH_table);
% >>>> get now pose
[ Info  ,  NowEulerAngle , InitialPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
FinalPosition = FinalPosition - [0, 0, Object.distance];

%% Path Planning


if (Scurve_Method.All)
    Option = Scurve_Method.PathAll;
else
    Option = Scurve_Method.P2;
end

switch Option  % 0: original, 1: time sync, 2: Jerk
    case 0
        % >>>> parameter
        acc_lim = 100;       % (m/s^2)
        vec_lim = 50;       % (m/s)
        acc_avg = 0.75;
        
        [ Cartesian_Cmd , Time2 ] = Scurve ( InitialPosition , FinalPosition ,  acc_lim , acc_avg , vec_lim , SamplingTime);
    case 1
        % >>>> parameter
        acc_lim = [100,100,100];       % (m/s^2)
        vec_lim = [50,50,50];       % (m/s)
        Scurve_j.Clear;
        Scurve_j.Input(   InitialPosition, FinalPosition, ...
            vec_lim,  acc_lim, ...
            acc_lim * 10, acc_lim * 100, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_Sync;
        [ Cartesian_Cmd , Time2 ] = Scurve_j.Get_Command;
    case 2
        % >>>> parameter
        acc_lim = [100,100,100];       % (m/s^2)
        vec_lim = [50,50,50];       % (m/s)
        Scurve_j.Clear;
        Scurve_j.Input(   InitialPosition, FinalPosition, ...
            vec_lim,  acc_lim, ...
            acc_lim * 10, acc_lim * 100, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_optimial_Cmd;
        [ Cartesian_Cmd , Time2 ] = Scurve_j.Get_Command;
end

Controller.velocity.error = [];

%%  Simulation Process

% >>>> Catesian space Simulation process
for i = 1 : length( Time2 )
    
    % >>>> 
    [ Info ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % Inverse Kinemetics calculate angle
    [ InverseAngle , SingularFlag2 ] = InverseKinemetics( FinalPose , Cartesian_Cmd(1:3,i)' , DH_table ) ;
    OptimalSol   = FindOptSol( InverseAngle , Robot.pos' ) ;
    
    % >>>> 基於Robot Jacobian軌跡
    [ RobotJacobian ] = ComputeRobotJacobian( DH_table , Robot.pos' );
    JointVelocity =  inv( RobotJacobian )  * [ Cartesian_Cmd(4,i) ; Cartesian_Cmd(5,i) ; Cartesian_Cmd(6,i) ; 0 ; 0 ; 0 ] ; 
    
    % >>>> Controller
    [Robot, Controller] = Control(Robot, Controller, [OptimalSol', JointVelocity], SamplingTime);
    
    % >>>> Check Object
    if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
        Object.Suction = true;
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
    Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           OptimalSol'];
    Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             JointVelocity];
    
end
end

if (PLOT.path > 2)
[Robot, Record] = Robot_delay(Robot, Object, delay(2), Controller, SamplingTime, Record, DH_table);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Catesian Scurve 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% >>>> get now pose
[ Info  ,  NowEulerAngle , InitialPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
FinalPosition = FinalPosition + [0, 0, Object.distance*2];

%% Path Planning
if (Scurve_Method.All)
    Option = Scurve_Method.PathAll;
else
    Option = Scurve_Method.P3;
end

switch Option  % 0: original, 1: time sync, 2: Jerk
    case 0
        % >>>> parameter
        acc_lim = 100;       % (m/s^2)
        vec_lim = 50;       % (m/s)
        acc_avg = 0.75;
        
        [ Cartesian_Cmd , Time3 ] = Scurve ( InitialPosition , FinalPosition ,  acc_lim , acc_avg , vec_lim , SamplingTime);
    case 1
        % >>>> parameter
        acc_lim = [100,100,100];       % (m/s^2)
        vec_lim = [50,50,50];       % (m/s)
        Scurve_j.Clear;
        Scurve_j.Input(   InitialPosition, FinalPosition, ...
            vec_lim,  acc_lim, ...
            acc_lim * 10, acc_lim * 100, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_Sync;
        [ Cartesian_Cmd , Time3 ] = Scurve_j.Get_Command;
    case 2
        % >>>> parameter
        acc_lim = [100,100,100];       % (m/s^2)
        vec_lim = [50,50,50];       % (m/s)
        Scurve_j.Clear;
        Scurve_j.Input(   InitialPosition, FinalPosition, ...
            vec_lim,  acc_lim, ...
            acc_lim * 10, acc_lim * 100, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_optimial_Cmd;
        [ Cartesian_Cmd , Time3 ] = Scurve_j.Get_Command;
end

Controller.velocity.error = [];

%%  Simulation Process

% >>>> Catesian space Simulation process
for i = 1 : length( Time3 )
    
    % >>>> 
    [ Info ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % Inverse Kinemetics calculate angle
    [ InverseAngle , SingularFlag2 ] = InverseKinemetics( FinalPose , Cartesian_Cmd(1:3,i)' , DH_table ) ;
    OptimalSol   = FindOptSol( InverseAngle , Robot.pos' ) ;
    
    % >>>> 基於Robot Jacobian軌跡
    [ RobotJacobian ] = ComputeRobotJacobian( DH_table , Robot.pos' );
    JointVelocity =  inv( RobotJacobian )  * [ Cartesian_Cmd(4,i) ; Cartesian_Cmd(5,i) ; Cartesian_Cmd(6,i) ; 0 ; 0 ; 0 ] ; 
    
    % >>>> Controller
    [Robot, Controller] = Control(Robot, Controller, [OptimalSol', JointVelocity], SamplingTime);
    
    % >>>> Check Object
    if (abs(sqrt(sum((C_NowPosition - Object.Center) .^2)) - Object.Width(3)/2) < Object.thereshould )
        Object.Suction = true;
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
    Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           OptimalSol'];
    Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             JointVelocity];
    
end
end
if (PLOT.path > 3)
[Robot, Record] = Robot_delay(Robot, Object, delay(3), Controller, SamplingTime, Record, DH_table);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint Scurve 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% >>>> final pose
FinalPose       = [0, 0, 180];
FinalPosition   = [Object.Goal, Object.Width(3) - 27];

% Inverse Kinemetics calculate angle
[ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( FinalPose , FinalPosition , DH_table ) ;

% >>>> find best angle
OptimalSol_Final   = FindOptSol( InverseJointAngle_Final   , Home_pose ) ;
Controller.velocity.error = [];

%% Path Planning

if (Scurve_Method.All)
    Option = Scurve_Method.PathAll;
else
    Option = Scurve_Method.P4;
end

switch Option  % 0: original, 1: time sync, 2: Jerk
    case 0
        % >>>> parameter
        acc_avg = 0.75;
        [ JointCmd , Time4 ] = Scurve_MultiAxis ( Robot.pos' , OptimalSol_Final , SamplingTime , acc_avg , ITRI_Limitation);
    case 1
        Scurve_j.Clear;
        Scurve_j.Input(   Robot.pos' , OptimalSol_Final , ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_Sync;
        [ JointCmd , Time4 ] = Scurve_j.Get_Command;
    case 2
        Scurve_j.Clear;
        Scurve_j.Input(   Robot.pos' , OptimalSol_Final , ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_optimial_Cmd;
        [ JointCmd , Time4 ] = Scurve_j.Get_Command;
end

Controller.velocity.error = [];
 
%% Simulation Process

% >>>> Joint space Scurve Simulation process
for i = 1 : length( Time4 )

    % >>>> Robot Kinetamics
    [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
      
    % >>>> Controller
    [Robot, Controller] = Control(Robot, Controller, [JointCmd(1:6,i),JointCmd(7:12,i)], SamplingTime);
    
    % >>>> Check Object
    if (i == length(Time4))
        Object.Suction = false;
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
end
if (PLOT.path > 4)
[Robot, Record] = Robot_delay(Robot, Object, delay(4), Controller, SamplingTime, Record, DH_table);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint Scurve 5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Path Planning

if (Scurve_Method.All)
    Option = Scurve_Method.PathAll;
else
    Option = Scurve_Method.P5;
end

switch Option  % 0: original, 1: time sync, 2: Jerk
    case 0
        % >>>> parameter
        acc_avg = 0.75;
        [ JointCmd , Time5 ] = Scurve_MultiAxis ( Robot.pos' , Home_pose , SamplingTime , acc_avg , ITRI_Limitation);
    case 1
        Scurve_j.Clear;
        Scurve_j.Input(   Robot.pos' , Home_pose , ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_Sync;
        [ JointCmd , Time5 ] = Scurve_j.Get_Command;
    case 2
        Scurve_j.Clear;
        Scurve_j.Input(   Robot.pos' , Home_pose , ...
            ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
            ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
            sqrt(3)/2, SamplingTime);
        Scurve_j.MultiAxis_Time_optimial_Cmd;
        [ JointCmd , Time5 ] = Scurve_j.Get_Command;
end

Controller.velocity.error = [];

%% Simulation Process

% >>>> Joint space Scurve Simulation process
for i = 1 : length( Time5 )

    % >>>> Robot Kinetamics
    [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
    
    % >>>> Controller
    [Robot, Controller] = Control(Robot, Controller, [JointCmd(1:6,i),JointCmd(7:12,i)], SamplingTime);
    
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
end
%% Compute Error & Plot

if (PLOT.robot.visible)
    % >>>> Show Trajectory
    figure('name','ShowTrajectory');

    % for i = 1 : 20 : (length( Time1 ) +length (Time2))
    switch PLOT.path
        case 1
            len = length( Time1 );
        case 2
            len = length( Time1 ) + delay(1) + length(Time2) ;
        case 3
            len = length( Time1 ) + delay(1) + length(Time2) + delay(2) + length(Time3);
        case 4
            len = length( Time1 ) + delay(1) + length(Time2) + delay(2) + length(Time3) + delay(3) + length(Time4) ;
        case 5
            len = length( Time1 ) + delay(1) + length(Time2) + delay(2) + length(Time3) + delay(3) + length(Time4) + delay(4) + length(Time5);
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

%% Joint record
if (PLOT.record.visible)
    switch PLOT.path
        case 1
            Time = [    Time1 ];
            time_record = [length(Time1)];
        case 2
            Time = [    Time1 ,...
                        ones(1,delay(1)) * SamplingTime + Time1(end),... 
                        Time2 + (Time1(end) + delay(1) * SamplingTime)];
            time_record = [length(Time1), delay(1), length(Time2)];
        case 3
            Time = [    Time1 ,...
                        ones(1,delay(1)) * SamplingTime + Time1(end),... 
                        Time2 + (Time1(end) + delay(1) * SamplingTime), ...
                        ones(1,delay(2)) * SamplingTime + (Time1(end) + Time2(end) + delay(1) * SamplingTime),... 
                        Time3 + (Time1(end)) + (Time2(end) + sum(delay(1:2)) * SamplingTime)];
            time_record = [length(Time1), delay(1), length(Time2), delay(2), length(Time3)];
        case 4
            Time = [    Time1 ,...
                        ones(1,delay(1)) * SamplingTime + Time1(end),... 
                        Time2 + (Time1(end) + delay(1) * SamplingTime), ...
                        ones(1,delay(2)) * SamplingTime + (Time1(end) + Time2(end) + delay(1) * SamplingTime),... 
                        Time3 + (Time1(end)) + (Time2(end) + sum(delay(1:2)) * SamplingTime), ...
                        ones(1,delay(3)) * SamplingTime + (Time1(end) + Time2(end) +Time3(end) + sum(delay(1:2)) * SamplingTime),...
                        Time4 + (Time1(end)) + (Time2(end) + Time3(end) + sum(delay(1:3)) * SamplingTime)];
            time_record = [length(Time1), delay(1), length(Time2), delay(2), length(Time3), delay(3), length(Time4)];
        case 5
            Time = [    Time1 ,...
                        ones(1,delay(1)) * SamplingTime + Time1(end),... 
                        Time2 + (Time1(end) + delay(1) * SamplingTime), ...
                        ones(1,delay(2)) * SamplingTime + (Time1(end) + Time2(end) + delay(1) * SamplingTime),... 
                        Time3 + (Time1(end)) + (Time2(end) + sum(delay(1:2)) * SamplingTime), ...
                        ones(1,delay(3)) * SamplingTime + (Time1(end) + Time2(end) +Time3(end) + sum(delay(1:2)) * SamplingTime),...
                        Time4 + (Time1(end)) + (Time2(end) + Time3(end) + sum(delay(1:3)) * SamplingTime) ,...
                        ones(1,delay(4)) * SamplingTime + (Time1(end) + Time2(end) +Time3(end) +Time4(end) + sum(delay(1:3)) * SamplingTime),...
                        Time5 + (Time1(end)) + (Time2(end) + Time3(end) + Time4(end) + sum(delay(1:4)) * SamplingTime)];
            time_record = [length(Time1), delay(1), length(Time2), delay(2), length(Time3), delay(3), length(Time4), delay(4), length(Time5)];
    end

    Record = PlotJointData(Record, Time, time_record, PLOT);


end
%% Check Performance
for i = 1:6
    Position_error = Record.Joint.JointCmd(i,:) - Record.Joint.JointRecord(:,i)';
    Velocity_error = Record.Joint.VelCmd(i,:) - Record.Performance.Derivate(:,i + 6)';
    Record.Performance.Position = [Record.Performance.Position; norm(Position_error)];
    Record.Performance.Velocity = [Record.Performance.Velocity; norm(Velocity_error)];
end
%%
file_name = 'Record';
switch Option
    case 0
        file_name = [file_name, '_Original'];
    case 1
        file_name = [file_name, '_Time_sync'];
    case 2
        file_name = [file_name, '_Minimize_Jerk'];
end
switch Controller.type
    case 'velocity'
        file_name = [file_name, '_Velocity'];
    case 'position'
        file_name = [file_name, '_Position'];
end
file_name = ['Record/', file_name, '.mat'];
save(file_name,'Record');


