function [ J2 , J2p , J3 , J3p ] = calculateJ2J3( Ex , Ey , Ez ,J1, DH_table )

theta = DH_table( : , 1 ) ;
d     = DH_table( : , 2 ) ;
a     = DH_table( : , 3 ) ;
alpha = DH_table( : , 4 ) ;

T1 = calculateT( J1, alpha(1), a(1), d(1) );
A1 = T1;
Bx = A1(1,4);
By = A1(2,4);
Bz = A1(3,4);

P = 0;
AB = a(1);
if(abs(Bx)>1)
    if (Ex*Bx < 0)
        
        P = -sqrt(Ex*Ex+Ey*Ey) - AB;
        
    else
        
        P = sqrt(Ex*Ex+Ey*Ey) - AB;
    end
    
elseif(abs(By)>1)
    if (Ey*By < 0)
        
        P = -sqrt(Ex*Ex+Ey*Ey) - AB;
        
    else
        
        P = sqrt(Ex*Ex+Ey*Ey) - AB;
    end
    
else
    
    P = sqrt(Ex*Ex+Ey*Ey) - AB;
end

OA = d(1);
Q = Ez - OA;
BC = a(2);
CD = a(3);
DE = d(4);
BE = sqrt(P*P+Q*Q);%sqrt((Ex-Bx)*(Ex-Bx)+(Ey-By)*(Ey-By)+(Ez-Bz)*(Ez-Bz));//
CE = sqrt(CD*CD+DE*DE);
angleCBE = acos((BC*BC+BE*BE-CE*CE)/(2*BC*BE));

angleEBU = atan2(Q,P);% 使用arctan2函數以修正arctan給出的角度值

J2=- ( pi/2-(angleCBE+angleEBU));
J2p=- ( pi/2-(angleEBU-angleCBE) );
% 增加一個機制 若超過180度 自動變負號
J2 = over180_rad(J2);
J2p = over180_rad(J2p);

angleBCE = acos((BC*BC+CE*CE-BE*BE)/(2*BC*CE));
angleECD = acos(CD/CE);%acosd((CD^2+CE^2-DE^2)/(2*CD*CE));
J3 = angleBCE + angleECD -pi ; % elbow up
J3p = pi -(angleBCE - angleECD); % elbow down
% 增加一個機制 若超過180度 自動變負號
J3 = over180_rad(J3);
J3p = over180_rad(J3p);


end