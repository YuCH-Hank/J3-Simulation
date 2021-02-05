function [ JointAngle , SingularFlag ] = InverseKinemetics( EulerAngle , Position , DH_table  )

SingularFlag = 0 ; % 看看是否在奇異點 

% 讀取DH參數
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

z1 = nsdt(1,3);
z2 = nsdt(2,3);
z3 = nsdt(3,3);
x1 = nsdt(1,1);
x2 = nsdt(2,1);
x3 = nsdt(3,1);
EEx = nsdt(1,4);
EEy = nsdt(2,4);
EEz = nsdt(3,4);    

% -----------------------------------------------

% Joint 1
J1s = zeros(1,2) ;
Ex = EEx - d(6)*z1;
Ey = EEy - d(6)*z2;
Ez = EEz - d(6)*z3;

[ J1s(1) , J1s(2) ] = calculateJ1(Ex,Ey,Ez);
  
% Joint 2, Joint 3
J2s = zeros(1,4) ; J3s = zeros(1,4) ;
[ J2s(1) , J2s(2) , J3s(1) , J3s(2) ] = calculateJ2J3(Ex,Ey,Ez,J1s(1) ,DH_table);
[ J2s(3) , J2s(4) , J3s(3) , J3s(4) ] = calculateJ2J3(Ex,Ey,Ez,J1s(2), DH_table);

% Joint 4, Joint 5, Joint 6
J4s = zeros(1,8) ; J5s = zeros(1,8) ; J6s = zeros(1,8) ;

[ J4s(1) , J4s(2) , J5s(1) , J5s(2) , J6s(1) , J6s(2) ] = calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s(1),J2s(1),J3s(1) ,DH_table );
[ J4s(3) , J4s(4) , J5s(3) , J5s(4) , J6s(3) , J6s(4) ] = calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s(1),J2s(2),J3s(2) ,DH_table );

J1s = real(J1s) ;J2s = real(J2s) ; J3s = real(J3s) ; J4s = real(J4s) ;

[ J4s(5) , J4s(6) , J5s(5) , J5s(6) , J6s(5) , J6s(6) ] = calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s(2),J2s(3),J3s(3) ,DH_table );
[ J4s(7) , J4s(8) , J5s(7) , J5s(8) , J6s(7) , J6s(8) ] = calculateJ4J5J6(EEx,EEy,EEz,x1,x2,x3,J1s(2),J2s(4),J3s(4) ,DH_table );

J5s = real(J5s) ; J6s = real(J6s) ;

JointAngle(1,1) = J1s(1);
JointAngle(1,2) = J2s(1);
JointAngle(1,3) = J3s(1);
JointAngle(1,4) = J4s(1);
JointAngle(1,5) = J5s(1);
JointAngle(1,6) = J6s(1);

JointAngle(2,1) = J1s(1);
JointAngle(2,2) = J2s(1);
JointAngle(2,3) = J3s(1);
JointAngle(2,4) = J4s(2);
JointAngle(2,5) = J5s(2);
JointAngle(2,6) = J6s(2);

JointAngle(3,1) = J1s(1);
JointAngle(3,2) = J2s(2);
JointAngle(3,3) = J3s(2);
JointAngle(3,4) = J4s(3);
JointAngle(3,5) = J5s(3);
JointAngle(3,6) = J6s(3);

JointAngle(4,1) = J1s(1);
JointAngle(4,2) = J2s(2);
JointAngle(4,3) = J3s(2);
JointAngle(4,4) = J4s(4);
JointAngle(4,5) = J5s(4);
JointAngle(4,6) = J6s(4);

JointAngle(5,1) = J1s(2);
JointAngle(5,2) = J2s(3);
JointAngle(5,3) = J3s(3);
JointAngle(5,4) = J4s(5);
JointAngle(5,5) = J5s(5);
JointAngle(5,6) = J6s(5);

JointAngle(6,1) = J1s(2);
JointAngle(6,2) = J2s(3);
JointAngle(6,3) = J3s(3);
JointAngle(6,4) = J4s(6);
JointAngle(6,5) = J5s(6);
JointAngle(6,6) = J6s(6);

JointAngle(7,1) = J1s(2);
JointAngle(7,2) = J2s(4);
JointAngle(7,3) = J3s(4);
JointAngle(7,4) = J4s(7);
JointAngle(7,5) = J5s(7);
JointAngle(7,6) = J6s(7);

JointAngle(8,1) = J1s(2);
JointAngle(8,2) = J2s(4);
JointAngle(8,3) = J3s(4);
JointAngle(8,4) = J4s(8);
JointAngle(8,5) = J5s(8);
JointAngle(8,6) = J6s(8);

    
end