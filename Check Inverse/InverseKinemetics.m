function [ JointAngle , SingularFlag ] = InverseKinemetics( EulerAngle , Position , DH_table  )

SingularFlag = false ; % 看看是否在奇異點

% >>>> 讀取DH參數
theta = DH_table( : , 1 ) ;
d     = DH_table( : , 2 ) ;
a     = DH_table( : , 3 ) ;
alpha = DH_table( : , 4 ) ;

%--------calculate wrist center position-----------------------------------

TEEFPosture  = [ EulerAngle(1) , EulerAngle(2) , EulerAngle(3) ];
EEF_Matr = RPY2Rot(TEEFPosture(1), TEEFPosture(2), TEEFPosture(3));

nsdt = [ EEF_Matr(1,1)      EEF_Matr(1,2)       EEF_Matr(1,3)         Position(1)  ;
         EEF_Matr(2,1)      EEF_Matr(2,2)       EEF_Matr(2,3)         Position(2)  ;
         EEF_Matr(3,1)      EEF_Matr(3,2)       EEF_Matr(3,3)         Position(3)  ;
         0                  0                   0                     1   ]   ;

 z = nsdt(1:3, 3)';
 x = nsdt(1:3, 1)';

% -----------------------------------------------

%% Joint 1
J1s = zeros(1,2) ;


Point_EE = Set(nsdt(1:3, 4)'); % (x, y, z)
Point_E  = minus(Point_EE,  d(6) * [z(1), z(2), z(3)]);

% Left Shoudler; Right Shoudler
[ J1s(1),        J1s(2)         ] = calculateJ1;


    %% Joint 2, Joint 3
    J2s = zeros(1,4) ; J3s = zeros(1,4) ;
    % elbow up, elbow down, elbow up,   elbow down
    [ J2s(1) ,  J2s(2) ,    J3s(1) ,    J3s(2) ] = calculateJ2J3( J1s(1) );
    [ J2s(3) ,  J2s(4) ,    J3s(3) ,    J3s(4) ] = calculateJ2J3( J1s(2) );
    
if (SingularFlag)
    JointAngle = zeros(8,6);
else
    %% Joint 4, Joint 5, Joint 6
    J4s = zeros(1,8) ; J5s = zeros(1,8) ; J6s = zeros(1,8) ;
    
    % non-flip
    [ J4s(1) , J4s(2) , J5s(1) , J5s(2) , J6s(1) , J6s(2) ] = calculateJ4J5J6( J1s(1), J2s(1), J3s(1)  );
    % flip
    [ J4s(3) , J4s(4) , J5s(3) , J5s(4) , J6s(3) , J6s(4) ] = calculateJ4J5J6( J1s(1), J2s(2), J3s(2)  );
    
    
    % non-flip
    [ J4s(5) , J4s(6) , J5s(5) , J5s(6) , J6s(5) , J6s(6) ] = calculateJ4J5J6( J1s(2), J2s(3), J3s(3)  );
    % flip
    [ J4s(7) , J4s(8) , J5s(7) , J5s(8) , J6s(7) , J6s(8) ] = calculateJ4J5J6( J1s(2), J2s(4), J3s(4)  );
    
    JointAngle = [  J1s(1), J2s(1), J3s(1), J4s(1), J5s(1), J6s(1);
                    J1s(1), J2s(1), J3s(1), J4s(2), J5s(2), J6s(2);
                    J1s(1), J2s(2), J3s(2), J4s(3), J5s(3), J6s(3);
                    J1s(1), J2s(2), J3s(2), J4s(4), J5s(4), J6s(4);
                    J1s(2), J2s(3), J3s(3), J4s(5), J5s(5), J6s(5);
                    J1s(2), J2s(3), J3s(3), J4s(6), J5s(6), J6s(6);
                    J1s(2), J2s(4), J3s(4), J4s(7), J5s(7), J6s(7);
                    J1s(2), J2s(4), J3s(4), J4s(8), J5s(8), J6s(8);];
end
%% Functions
    function Rzyx = RPY2Rot(A, B, C)
        %--------------------------------------------------------------------------------------------------------------------------
        %------ Deg to Rad ---------------------------------------------
        DegToRad = pi/180;
        %------ Deg to Rad ---------------------------------------------
        Alpha = A*DegToRad;
        Beta  = B*DegToRad;
        Gamma = C*DegToRad;
        %------ 3x(3) matrix ----------------------------------------------
        Rzyx = [    cos(Alpha)*cos(Beta),  cos(Alpha)*sin(Beta)*sin(Gamma) - cos(Gamma)*sin(Alpha),    sin(Alpha)*sin(Gamma) + cos(Alpha)*cos(Gamma)*sin(Beta) ;
                    cos(Beta)*sin(Alpha),  cos(Alpha)*cos(Gamma) + sin(Alpha)*sin(Beta)*sin(Gamma),    cos(Gamma)*sin(Alpha)*sin(Beta) - cos(Alpha)*sin(Gamma) ;
                    -sin(Beta),             cos(Beta)*sin(Gamma),                                       cos(Beta)*cos(Gamma) ];
    end

    function Angle = over180_rad(input)
        if (input > pi)
            input = input - 2*pi;
        elseif (input<-pi)
            input = input + 2*pi;
        end
        Angle = input;
    end

    function [ J1 , J1p ] = calculateJ1
        J1  = atan2(Point_E.y, Point_E.x);
        J1p = atan2(Point_E.y, Point_E.x) + pi;
        
        % 增加一個機制 若超過180度 自動變負號
        J1  = over180_rad(J1);
        J1p = over180_rad(J1p);
        
    end

    function [ J2 , J2p , J3 , J3p ] = calculateJ2J3( J1 )
        T1 = calculateT( J1, alpha(1), a(1), d(1) );
        A1 = T1;
        
        Point_B = Set(A1(:,4)');
        
        P = 0;
        AB = a(1);
        
        % Point_E, Point_B Coordinate O in Didderent Quadrant
        if ( Point_E.x * Point_B.x < 0)
            % E in Coordinate B third coordinate
            P = -sqrt( Point_E.x^2 + Point_E.y^2 ) - AB;
            
            % Point_E, Point_B Coordinate O in Same Quadrant
        else
            % E in Coordinate B third coordinate
            P = sqrt( Point_E.x^2 + Point_E.y^2 ) - AB;
            
        end
        
        OA = d(1);
        Q = Point_E.z - OA;
        BC = a(2);
        CD = a(3);
        DE = d(4);
        BE = sqrt(P^2+Q^2);
        CE = sqrt(CD^2+DE^2);
        
        % Check Singular
        if ((BC + CE < BE ) || (CE - BC > BE))
            J2 = 0 ; J2p = 0; J3 = 0 ; J3p = 0;
            SingularFlag = true;
            return;
        end
        
        angleCBE = acos((BC^2+BE^2-CE^2)/(2*BC*BE));
        
        angleEBU = atan2(Q,P);
        
        J2  = - pi/2 + (angleCBE + angleEBU);
        J2p = - pi/2 + angleEBU - angleCBE ;
        
        % 增加一個機制 若超過180度 自動變負號
        J2 = over180_rad(J2);
        J2p = over180_rad(J2p);
        
        %
        angleBCE = acos((BC ^ 2 + CE ^ 2 - BE^2)/(2*BC*CE));
        angleECD = acos(CD/CE);
        J3 = angleBCE + angleECD - pi ; % elbow up
        J3p = pi - angleBCE + angleECD; % elbow down
        % 增加一個機制 若超過180度 自動變負號
        J3  = over180_rad(J3);
        J3p = over180_rad(J3p);
    end

    function [ J4 , J4p , J5 , J5p , J6 , J6p ] = calculateJ4J5J6( J1, J2, J3 )
        
        T1 = calculateT( J1, alpha(1), a(1), d(1) );
        T2 = calculateT( J2 + (pi/2) , alpha(2), a(2), d(2) );
        T3 = calculateT( J3, alpha(3), a(3), d(3) );
        A3 = T1 * T2 * T3 ;
        
        newPoint_EE.x = Point_EE.x - A3(1,4);
        newPoint_EE.y = Point_EE.y - A3(2,4);
        newPoint_EE.z = Point_EE.z - A3(3,4);
        
        
        xProjection = newPoint_EE.x * A3(1,1) + newPoint_EE.y * A3(2,1) + newPoint_EE.z * A3(3,1);
        yProjection = newPoint_EE.x * A3(1,2) + newPoint_EE.y * A3(2,2) + newPoint_EE.z * A3(3,2);
        zProjection = newPoint_EE.x * A3(1,3) + newPoint_EE.y * A3(2,3) + newPoint_EE.z * A3(3,3);
        
        J4 = atan2(yProjection,xProjection);
        J4p = atan2(yProjection,xProjection) + pi;
        % 增加一個機制 若超過180度 自動變負號
        J4 = over180_rad(J4);
        J4p = over180_rad(J4p);
        
        Point_EE.xdir1 = xProjection * A3(1,1);
        Point_EE.xdir2 = xProjection * A3(2,1);
        Point_EE.xdir3 = xProjection * A3(3,1);
        
        Point_EE.ydir1 = yProjection * A3(1,2);
        Point_EE.ydir2 = yProjection * A3(2,2);
        Point_EE.ydir3 = yProjection * A3(3,2);
        
        EEinXYprojection1 = Point_EE.xdir1 + Point_EE.ydir1;
        EEinXYprojection2 = Point_EE.xdir2 + Point_EE.ydir2;
        EEinXYprojection3 = Point_EE.xdir3 + Point_EE.ydir3;
        
        EEinXYprojectionValue = sqrt(EEinXYprojection1 ^ 2 + EEinXYprojection2 ^ 2 + EEinXYprojection3 ^ 2 );
        
        
        % 會用102 是因為末端長度是102，如果用newEEvalue，那是末端長度和前一軸長度相加
        J5temp1cos = (zProjection - d(4))/d(6);
        J5temp2cos = (  newPoint_EE.x * EEinXYprojection1 + ...
                        newPoint_EE.y * EEinXYprojection2 + ...
                        newPoint_EE.z * EEinXYprojection3) / (d(6) * EEinXYprojectionValue);
        
        
        if (J5temp1cos>1)
            J5temp1cos = 1;
        end
        
        if (J5temp2cos>1)
            J5temp2cos = 1;
        end
        
        if (J5temp1cos<-1)
            J5temp1cos = -1;
        end
        
        if (J5temp2cos<-1)
            J5temp2cos = -1;
        end
        
        % 如果有算術誤差，將其消除
        J5temp1 = acos(J5temp1cos);
        J5temp2 = acos(J5temp2cos);
        
        if (J5temp2 > pi/2)
            %     J5 = -1*J5temp1;
            J5 = J5temp1;
        else
            J5 = J5temp1;
            
            J5p = -J5;
            % 增加一個機制 若超過180度 自動變負號
            J5 = over180_rad(J5);
            J5p = over180_rad(J5p);
            
            T4 = calculateT( J4, alpha(4), a(4), d(4) );
            T5 = calculateT( J5, alpha(5), a(5), d(5) );
            
            
            A5 = T1 * T2 * T3 * T4 * T5 ;
            
            
            J6temp1cos = x(1) * A5(1,1) + x(2) * A5(2,1) + x(3) * A5(3,1);
            J6temp2cos = x(1) * A5(1,2) + x(2) * A5(2,2) + x(3) * A5(3,2);
            
            if (J6temp1cos > 1)
                J6temp1cos = 1;
            end
            
            if (J6temp2cos > 1)
                J6temp2cos = 1;
            end
            
            if (J6temp1cos < -1)
                J6temp1cos = -1;
            end
            
            if (J6temp2cos < -1)
                J6temp2cos = -1;
            end
            
            % 如果有算術誤差，將其消除
            J6temp1 = acos(J6temp1cos);
            J6temp2 = acos(J6temp2cos);
            
            if (J6temp2 > pi/2)
                
                J6 = -1 * J6temp1;
                J6p = -1 * J6temp1 + pi;
                
            else
                
                J6 = J6temp1;
                J6p = J6temp1 + pi;
                
            end
            
            % 增加一個機制 若超過180度 自動變負號
            J6 = over180_rad(J6);
            J6p = over180_rad(J6p);
        end
    end

    function T = calculateT( theta, alpha, a, d )
        RotZ = [    1,  0,              0,           0;
                    0,  cos(alpha),     -sin(alpha), 0;
                    0,  sin(alpha),     cos(alpha),  0;
                    0,  0,              0,           1];
        
        %-----------------------------------
        TransZ = [  1, 0, 0, a;
                    0, 1, 0, 0;
                    0, 0, 1, 0;
                    0, 0, 0, 1];
        
        %-----------------------------------
        TransX = [  1, 0, 0, 0;
                    0, 1, 0, 0;
                    0, 0, 1, d;
                    0, 0, 0, 1];
        
        %-----------------------------------
        RotX = [    cos(theta),     -sin(theta),    0,              0;
                    sin(theta),     cos(theta),     0,              0;
                    0,              0,              1,              0;
                    0,              0,              0,              1];
        
        
        T=RotX*TransX*TransZ*RotZ;
        
    end

    function Output = Set(pos)
        Output = struct('x', pos(1), 'y', pos(2), 'z', pos(3));
    end

    function Output = minus(Point, vector)
        Output.x = Point.x - vector(1);
        Output.y = Point.y - vector(2);
        Output.z = Point.z - vector(3);
    end
end