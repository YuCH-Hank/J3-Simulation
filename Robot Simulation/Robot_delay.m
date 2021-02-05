function [Robot, Record] = Robot_delay(Robot, Object, delay, Controller, SamplingTime, Record, DH_table)
    pos = Robot.pos;
    for i = 1:delay
        % >>>> Robot Kinetamics
        [ Info  ,  C_NowEulerAngle , C_NowPosition ] = ForwardKinemetics( DH_table , Robot.pos' ) ;
        
        % >>>> Controller
        Controller.type = 'position';
        Robot = Control(Robot, Controller, [pos,zeros(6,1)], SamplingTime);
        
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
        Record.Joint.JointCmd                                               = [Record.Joint.JointCmd,           pos];
        Record.Joint.VelCmd                                                 = [Record.Joint.VelCmd,             zeros(length(pos),1)];
    end

end