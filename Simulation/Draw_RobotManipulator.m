%%  畫圖 畫機械手臂 輸入: 自由度，各關節的位置，各關節的座標
function Draw_RobotManipulator( JointPos, JointDir , Axis, Draw_Augmented)

% >>>> ITRI_parameter
[ ITRI_parameter , DH_table , SamplingTime ] = ITRI_Parameter;
DOF = size( DH_table , 1 ) ;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

% >>>> Color
Robot    = [ 0.6627 , 0.6627 , 0.6627 ];

% >>>> plot joint and axis
plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'linewidth', 2)
hold on;
plot3( JointPos(1,1:end), JointPos(2,1:end), JointPos(3,1:end),'ro','linewidth', 3);
hold on;

grid on;
xlabel('X軸');
ylabel('Y軸');
zlabel('Z軸');

axis( [ ITRI_Limitation.Cartesian.Pos(1,:) * 1.2 , ...
        ITRI_Limitation.Cartesian.Pos(2,:) * 1.5 , ...
        -78                                      , ITRI_Limitation.Cartesian.Pos(3,2)] );

PosLimit = [  80  -40  ;
              50  -50  ;
              100   0  ];
          
          
Xmax = PosLimit( 1 , 1 ) ;
Ymax = PosLimit( 2 , 1 ) ;
Zmax = PosLimit( 3 , 1 ) ;
Xmin = PosLimit( 1 , 2 ) ;
Ymin = PosLimit( 2 , 2 ) ;
Zmin = PosLimit( 3 , 2 ) ;       

if (Axis)
    for i = 1 : DOF+1

        %-----------------------------------------
        nUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 1 );
        nBase   = [JointPos( : , i ) nUnit_v];
        plot3(nBase(1,1:end),nBase(2,1:end),nBase(3,1:end),'r','linewidth',2);

        %--------------------------------------------
        sUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 2 );
        sBase   = [JointPos( : , i ) sUnit_v];
        plot3(sBase(1,1:end),sBase(2,1:end),sBase(3,1:end),'g','linewidth',2);

        %--------------------------------------------
        aUnit_v = JointPos( : , i ) + 10 * JointDir( : , 3 * (i-1) + 3 );
        aBase   = [JointPos( : , i ) aUnit_v];
        plot3(aBase(1,1:end),aBase(2,1:end),aBase(3,1:end),'b','linewidth',2);
        %--------------------------------------------
        hold on

    end 
end

if (Draw_Augmented)
    % >>>> Axis 1 >>>>
    Draw_Augment([], [], [], [0, 0, 0], [0, 0, 23], 23/2, Robot, 'Cylinder');

    Direction   = [JointDir( : , 3 * (2-1) + 3 )';
                  -JointDir( : , 3 * (2-1) + 1 )';
                  JointDir( : , 3 * (2-1) + 2 )';];
    start_point = JointPos(:,1)' + JointDir( : , 3 * (2-1) + 3 )' * -6.5 + JointDir( : , 3 * (1-1) + 3 )' * 23 + JointDir( : , 3 * (2-1) + 1 )' * 8;
    Width       = [13, 13, 17];
    Draw_Augment(start_point, Direction, Width, [], [], [], Robot, 'Triangle');

    % >>>> Axis 2 >>>>
    % >>>> Cylinder
    start_point = JointPos(:,2)' + JointDir( : , 3 * (2-1) + 3 )' * 13;
    end_point   = JointPos(:,2)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    start_point = JointPos(:,2)' - JointDir( : , 3 * (2-1) + 3 )' * 10;
    end_point   = JointPos(:,2)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    % >>>> Cube >>>>
    len         = sqrt(sum((JointPos(:,2) - JointPos(:,3)).^2));
    Direction   = [-(JointPos(:,2) - JointPos(:,3))'/len; 
                   cross(-(JointPos(:,2) - JointPos(:,3))'/len,JointDir( : , 3 * (2-1) + 3 )');
                   JointDir( : , 3 * (2-1) + 3 )';];
    start_point = JointPos(:,2)' - JointDir( : , 3 * (3-1) + 3 )' * 9 - Direction(2,:) * 5;
    Width       = [len, 10, 2];
    Draw_Augment(start_point, Direction, Width, [], [], [], Robot, 'Cube');

    % >>>> Axis 3 >>>>
    start_point = JointPos(:,3)' + JointDir( : , 3 * (3-1) + 3 )' * 13;
    end_point   = JointPos(:,3)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    start_point = JointPos(:,3)' - JointDir( : , 3 * (3-1) + 3 )' * 10;
    end_point = JointPos(:,3)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    % >>>> Axis 4 >>>>
    % >>>> Cylinder
    start_point = JointPos(:,4)' + JointDir( : , 3 * (4-1) + 3 )' * 18;
    end_point   = JointPos(:,4)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    start_point = JointPos(:,4)' - JointDir( : , 3 * (4-1) + 3 )' * 13;
    end_point = JointPos(:,4)';
    r           = 15/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    % >>>> Cube
    Direction   = [JointDir( : , 3 * (4-1) + 1 )';
                   JointDir( : , 3 * (4-1) + 3 )';
                   JointDir( : , 3 * (4-1) + 2 )';];
    start_point = JointPos(:,4)' + JointDir( : , 3 * (4-1) + 3 )' * 10 - JointDir( : , 3 * (4-1) + 1 )' * 2.5 + JointDir( : , 3 * (4-1) + 2 )' * 4;
    Width       = [5, 20, 2];
    Draw_Augment(start_point, Direction, Width, [], [], [], Robot, 'Cube');

    Direction   = [JointDir( : , 3 * (4-1) + 1 )';
                   JointDir( : , 3 * (4-1) + 3 )';
                  -JointDir( : , 3 * (4-1) + 2 )';];
    start_point = JointPos(:,4)' + JointDir( : , 3 * (4-1) + 3 )' * 10 - JointDir( : , 3 * (4-1) + 1 )' * 2.5 - JointDir( : , 3 * (4-1) + 2 )' * 4;
    Width       = [5, 20, 2];
    Draw_Augment(start_point, Direction, Width, [], [], [], Robot, 'Cube');

    % >>>> Axis 5 >>>>
    % >>>> Cylinder
    start_point = JointPos(:,5)' + JointDir( : , 3 * (6-1) + 3 )' * 8;
    end_point   = JointPos(:,5)';
    r           = 8/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');

    start_point = JointPos(:,5)' + -JointDir( : , 3 * (6-1) + 3 )' * 5;
    end_point   = JointPos(:,5)';
    r           = 8/2;
    Draw_Augment([], [], [], start_point, end_point, r, Robot, 'Cylinder');


    % >>>> Axis 6 >>>>
    % >>>> Cube
    Direction = [-JointDir( : , 3 * (7-1) + 1 )';
                  JointDir( : , 3 * (7-1) + 2 )';
                 -JointDir( : , 3 * (7-1) + 3 )';];
    start_point = JointPos(:,7)' + JointDir( : , 3 * (7-1) + 1 )' * 1 - JointDir( : , 3 * (7-1) + 2 )' * 1 ;
    Width       = [2, 2, 20];
    Draw_Augment(start_point, Direction, Width, [], [], [], Robot, 'Cube');
end


PosLimit = [  80  -40  ;
              50  -50  ;
              100   0  ];
          
Xmax = PosLimit( 1 , 1 ) ;
Ymax = PosLimit( 2 , 1 ) ;
Zmax = PosLimit( 3 , 1 ) ;
Xmin = PosLimit( 1 , 2 ) ;
Ymin = PosLimit( 2 , 2 ) ;
Zmin = PosLimit( 3 , 2 ) ;          

%-------------------------------------
Base = [ Xmax Ymax Zmax ; Xmax Ymin Zmax ; Xmin Ymin Zmax ; Xmin Ymax Zmax ] ;
f = [ 1 2 3 4 ] ;
patch('Faces',f ,'Vertices',Base,'EdgeColor','red','LineStyle','--','FaceColor','none')
hold on;
%-------------------------------------
Base = [ Xmax Ymax Zmax ; Xmax Ymin Zmax ; Xmax Ymin Zmin ; Xmax Ymax Zmin ] ;
f = [ 1 2 3 4 ] ;
patch('Faces',f ,'Vertices',Base,'EdgeColor','red','LineStyle','--','FaceColor','none')
hold on;
%-------------------------------------
Base = [ Xmax Ymax Zmax ; Xmin Ymax Zmax ; Xmin Ymax Zmin ; Xmax Ymax Zmin ] ;
f = [ 1 2 3 4 ] ;
patch('Faces',f ,'Vertices',Base,'EdgeColor','red','LineStyle','--','FaceColor','none')
hold on;
%-------------------------------------
Base = [ Xmin Ymin Zmax ; Xmin Ymax Zmax ; Xmin Ymax Zmin ; Xmin Ymin Zmin ] ;
f = [ 1 2 3 4 ] ;
patch('Faces',f ,'Vertices',Base,'EdgeColor','red','LineStyle','--','FaceColor','none')
hold on;
%-------------------------------------
Base = [ Xmax Ymin Zmax ; Xmin Ymin Zmax ; Xmin Ymin Zmin ; Xmax Ymin Zmin ] ;
f = [ 1 2 3 4 ] ;
patch('Faces',f ,'Vertices',Base,'EdgeColor','red','LineStyle','--','FaceColor','none')
hold on;      
          
end