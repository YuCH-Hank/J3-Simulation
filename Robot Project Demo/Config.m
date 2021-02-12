classdef Config
   properties
       Home_pose = NaN;
       Object = NaN;
       PLOT = NaN;
       Record = NaN;
       Robot = NaN;
       RL = NaN;
       Attractive = NaN;
       Scurve_Method = NaN;
   end
   methods
       function obj = Config(type) 
           switch type
               case 'Vrep'
                    obj = obj.Config_Vrep;
                    
               case 'RL'
                   obj = obj.Config_RL;
                   
               case 'Experiment'
                   obj = obj.Config_Experiment;
               
           end
       end
       
       function [Object, PLOT, Robot, Record, Home_pose, RL, Attractive, Scurve_Method] = Get_Config(obj)
           Object       = obj.Object;
           PLOT         = obj.PLOT;
           Robot        = obj.Robot;
           Record       = obj.Record;
           Home_pose    = obj.Home_pose;
           RL           = obj.RL;
           Attractive   = obj.Attractive;
           Scurve_Method = obj.Scurve_Method;
       end
       
       function obj = Config_Vrep(obj)
           % >>>> Home pose
            obj.Home_pose   = [  0   0   0   0   0   0 ] * pi / 180;

            % >>>> draw object
            obj.Object      = struct(   'Width'         , [10,  10,  20],...
                                        'Center'        , [50, -20,  20 * 0.5 - 27],...
                                        'Color'         , 'b',...
                                        'distance'      , 10,...
                                        'Suction'       , false,...
                                        'Goal'          , [30, 20],...
                                        'thereshould'   , 0.004);

            % >>>> plot
            obj.PLOT        = struct(   'robot'         , struct(   'visible'   ,   true,...
                                                                    'Augmented' ,   false,...
                                                                    'Axis'      ,   true),...
                                        'record'        , struct(   'visible'   ,   true,...
                                                                    'recorder'  ,   2,...
                                                                    'low_pass'  ,   true,...
                                                                    'low_pass_f',   50,...
                                                                    'time'      ,   true,...
                                                                    'Save_Plot' ,   true),...
                                        'path'          , 5);

            % >>>> Record
            obj.Record      = struct(   'Cartesian'     , struct(   'EEFRecord'     , [],...
                                                                    'CenterRecord'  , [],...
                                                                    'EulerAngle'    , []), ...
                                        'Joint'         , struct(   'JointRecord'   , [],...
                                                                    'JointPosRecord', [],...
                                                                    'JointDirRecord', [],...
                                                                    'JointCmd'      , [],...
                                                                    'VelCmd'        , []),...
                                        'vrep'          , struct(   'joint'         , []),...
                                        'Performance'   , struct(   'Position'      , [],...
                                                                    'Velocity'      , [],...
                                                                    'Derivate'      , []));

            obj.Scurve_Method = struct( 'P1'            , 0,...
                                        'P2'            , 0,...
                                        'P3'            , 0,...
                                        'P4'            , 0,...
                                        'P5'            , 0,...
                                        'All'           , true,...
                                        'PathAll'       , 0);       % 0: original, 1: time sync, 2: Jerk 

            % >>>> Robot
            obj.Robot       = struct(   'pos'           , obj.Home_pose',...
                                        'vec'           , zeros(length(obj.Home_pose) ,1),...
                                        'vec_old'       , zeros(length(obj.Home_pose) ,1),...
                                        'acc'           , zeros(length(obj.Home_pose) ,1),...
                                        'acc_old'       , zeros(length(obj.Home_pose) ,1),...
                                        'tor'           , zeros(length(obj.Home_pose) ,1));   
       end
       
       function obj = Config_RL(obj)
        % >>>> Home pose
        obj.Home_pose   = [  0   0   0   0   0   0 ] * pi / 180;

        % >>>> draw object
        obj.Object      = struct(   'Width'         , [10,  10,  20],...
                                    'Center'        , [50, -20,  20 * 0.5 - 27],...
                                    'Color'         , 'b',...
                                    'distance'      , 10,...
                                    'Suction'       , false,...
                                    'Goal'          , [30, 20],...
                                    'thereshould'   , 0.004);

        % >>>> plot
        obj.PLOT        = struct(   'robot'         , struct(   'visible'   ,   false,...
                                                                'Augmented' ,   false,...
                                                                'Axis'      ,   true),...
                                    'record'        , struct(   'visible'   ,   true,...
                                                                'recorder'  ,   2,...
                                                                'low_pass'  ,   true,...
                                                                'low_pass_f',   50,...
                                                                'time'      ,   true,...
                                                                'Save_Plot' ,   true),...
                                    'path'          , 1);

        % >>>> Record
        obj.Record      = struct(   'Cartesian'     , struct(   'EEFRecord'     , [],...
                                                                'CenterRecord'  , [],...
                                                                'EulerAngle'    , []), ...
                                    'Joint'         , struct(   'JointRecord'   , [],...
                                                                'JointPosRecord', [],...
                                                                'JointDirRecord', [],...
                                                                'JointCmd'      , [],...
                                                                'VelCmd'        , []),...
                                    'vrep'          , struct(   'joint'         , []),...
                                    'Performance'   , struct(   'Position'      , [],...
                                                                'Velocity'      , [],...
                                                                'Derivate'      , []));
        obj.Scurve_Method = struct( 'P1'            , 0,...
                                    'P2'            , 0,...
                                    'P3'            , 0,...
                                    'P4'            , 0,...
                                    'P5'            , 0,...
                                    'All'           , true,...
                                    'PathAll'       , 1);       % 0: original, 1: time sync, 2: Jerk 

        % >>>> Robot
        obj.Robot       = struct(   'pos'           , obj.Home_pose',...
                                    'vec'           , zeros(length(obj.Home_pose) ,1),...
                                    'vec_old'       , zeros(length(obj.Home_pose) ,1),...
                                    'acc'           , zeros(length(obj.Home_pose) ,1),...
                                    'acc_old'       , zeros(length(obj.Home_pose) ,1),...
                                    'tor'           , zeros(length(obj.Home_pose) ,1));

        % >>>> RL
        obj.RL          = struct(   'max_step_number'   ,   200,...
                                    'segment_size'      ,   0.01,...
                                    'thereshould'       ,   0.005,...
                                    'min_time'          ,   0.025,...
                                    'min'               ,   false);
        
        % >>>> attractive field
        obj.Attractive  = struct(   'Vmax'              ,   [ 0.15; 0.15; 0.15; 0.15; 0.15; 0.05],... % (cm/s)
                                    'Vmin'              ,   [ 0.00; 0.00; 0.00; 0.00; 0.00; 0.00 ]);   % (cm/s)
       end
       
       function obj = Config_Experiment(obj)
        % >>>> Home pose
        obj.Home_pose   = [  0   0   0   0   0   0 ] * pi / 180;

        % >>>> draw object
        obj.Object      = struct(   'Width'         , [10,  10,  20],...
                                    'Center'        , [50, -20,  20 * 0.5 - 27],...
                                    'Color'         , 'b',...
                                    'distance'      , 10,...
                                    'Suction'       , false,...
                                    'Goal'          , [30, 20],...
                                    'thereshould'   , 0.004);

        % >>>> plot
        obj.PLOT        = struct(   'robot'         , struct(   'visible'   ,   true,...
                                                                'Augmented' ,   false,...
                                                                'Axis'      ,   true),...
                                    'record'        , struct(   'visible'   ,   true,...
                                                                'recorder'  ,   4,...
                                                                'low_pass'  ,   true,...
                                                                'low_pass_f',   50,...
                                                                'time'      ,   true,...
                                                                'Save_Plot' ,   false),...
                                    'path'          , 5);
                                
        obj.Scurve_Method = struct( 'P1'            , 0,...
                                    'P2'            , 0,...
                                    'P3'            , 0,...
                                    'P4'            , 0,...
                                    'P5'            , 0,...
                                    'All'           , true,...
                                    'PathAll'       , 1);       % 0: original, 1: time sync, 2: Jerk 

        % >>>> Record
        obj.Record      = struct(   'Cartesian'     , struct(   'EEFRecord'     , [],...
                                                                'CenterRecord'  , [],...
                                                                'EulerAngle'    , []), ...
                                    'Joint'         , struct(   'JointRecord'   , [],...
                                                                'JointPosRecord', [],...
                                                                'JointDirRecord', [],...
                                                                'JointCmd'      , [],...
                                                                'VelCmd'        , []),...
                                    'vrep'          , struct(   'joint'         , []),...
                                    'Performance'   , struct(   'Position'      , [],...
                                                                'Velocity'      , [],...
                                                                'Derivate'      , []),...
                                    'Time'          , struct(   't'             , nan,...
                                                                'len'           , nan,...
                                                                'Segment'       , []),...
                                    'Command'       , struct(   'Initial'       , false,...
                                                                'Position'      , [],...
                                                                'Velocity'      , [],...
                                                                'count'         , 1,...
                                                                'length'        , []));


        % >>>> Robot
        obj.Robot       = struct(   'pos'           , obj.Home_pose',...
                                    'vec'           , zeros(length(obj.Home_pose) ,1),...
                                    'vec_old'       , zeros(length(obj.Home_pose) ,1),...
                                    'acc'           , zeros(length(obj.Home_pose) ,1),...
                                    'acc_old'       , zeros(length(obj.Home_pose) ,1),...
                                    'tor'           , zeros(length(obj.Home_pose) ,1));   
       end
       
       function Collection(obj)
           disp('>>>>>>>>>> Home Pose');
           fprintf('\t');fprintf('%.1f\t',obj.Home_pose');fprintf('\n\n');
           
           %            disp('>>>>>>>>>> Object');
           %            obj.Object
           
           if (isa(obj.Scurve_Method,'struct'))
               disp('>>>>>>>>>> Scurve_Method');
               switch(obj.Scurve_Method.PathAll)
                   case(0)
                       fprintf('\tOriginal\n');
                   case(1)
                       fprintf('\tTime Sync\n');
                   case(2)
                       fprintf('\tMinimize Jerk\n');
               end
           end
                      
           if (isa(obj.RL,'struct'))
               fprintf('\n');disp('>>>>>>>>>> RL');
               obj.RL
           end
           
           if (isa(obj.Attractive,'struct'))
               fprintf('\n');disp('>>>>>>>>>> Attractive');
               fprintf('\tVmax : ');fprintf('%.1f\t',obj.Attractive.Vmax');fprintf('\n');
               fprintf('\tVmin : ');fprintf('%.1f\t',obj.Attractive.Vmin');fprintf('\n');
           end
       end
   end
end