function [ J4 , J4p , J5 , J5p , J6 , J6p ] = calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1,J2,J3, DH_table )

theta = DH_table( : , 1 ) ;
d     = DH_table( : , 2 ) ;
a     = DH_table( : , 3 ) ;
alpha = DH_table( : , 4 ) ;

%% 解 Angle4  Angle5 Angle6

T1 = calculateT( J1, alpha(1), a(1), d(1) );
T2 = calculateT( J2 + (pi/2) , alpha(2), a(2), d(2) );
T3 = calculateT( J3, alpha(3), a(3), d(3) );
A3 = T1 * T2 * T3 ;

newEEx = EEx - A3(1,4);
newEEy = EEy - A3(2,4);
newEEz = EEz - A3(3,4);


xProjection = newEEx * A3(1,1) + newEEy * A3(2,1) + newEEz * A3(3,1);
yProjection = newEEx * A3(1,2) + newEEy * A3(2,2) + newEEz * A3(3,2);
zProjection = newEEx * A3(1,3) + newEEy * A3(2,3) + newEEz * A3(3,3);

J4 = atan2(yProjection,xProjection);
J4p = atan2(yProjection,xProjection) + pi;
% 增加一個機制 若超過180度 自動變負號
J4 = over180_rad(J4);
J4p = over180_rad(J4p);

EExdir1 = xProjection * A3(1,1);
EExdir2 = xProjection * A3(2,1);
EExdir3 = xProjection * A3(3,1);

EEydir1 = yProjection * A3(1,2);
EEydir2 = yProjection * A3(2,2);
EEydir3 = yProjection * A3(3,2);

EEinXYprojection1 = EExdir1 + EEydir1;
EEinXYprojection2 = EExdir2 + EEydir2;
EEinXYprojection3 = EExdir3 + EEydir3;

EEinXYprojectionValue = sqrt(EEinXYprojection1 * EEinXYprojection1 + ...
                             EEinXYprojection2 * EEinXYprojection2 + ...
                             EEinXYprojection3 * EEinXYprojection3);


% 會用102 是因為末端長度是102，如果用newEEvalue，那是末端長度和前一軸長度相加
J5temp1cos = (zProjection - d(4))/d(6);
J5temp2cos = (newEEx * EEinXYprojection1 + ...
              newEEy * EEinXYprojection2 + ...
              newEEz * EEinXYprojection3) / (d(6) * EEinXYprojectionValue);


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
    
    
    J6temp1cos = x1 * A5(1,1) + x2 * A5(2,1) + x3 * A5(3,1);
    J6temp2cos = x1 * A5(1,2) + x2 * A5(2,2) + x3 * A5(3,2);
    
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