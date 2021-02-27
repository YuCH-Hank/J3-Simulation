clc,clear,close all
% >>>> ITRI Parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

[ Info  ,  NowEulerAngle , IniPosition ] = ForwardKinemetics( DH_table , [0,0.5,0.2,0.3,0.1,0.4] ) ;

%%

%%
[ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( NowEulerAngle , [66.6666666666667,13.3333333333333,54.4444444444444] , DH_table ) ;

if (SingularFlag2)
    fprintf('S\n');
else
    
    for i = 1 : 8
        figure('name',sprintf('Pose %d',i));
        [ Info  ,  NowEulerAngle , Ini ] = ForwardKinemetics( DH_table , InverseJointAngle_Final(i,:)');
        
        Ini
        Draw_RobotManipulator(  Info.JointPos, ...
            Info.JointDir , ...
            true , false) ;
        Draw_Base();
        hold off;
    end
end