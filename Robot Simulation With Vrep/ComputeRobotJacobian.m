function[ Jacobian ] = ComputeRobotJacobian(  DH_table , JointAngle )


%% Parameter + Symbolic

s1 = sin( JointAngle(1,1) + DH_table(1,1) );
s2 = sin( JointAngle(1,2) + DH_table(2,1) );
s3 = sin( JointAngle(1,3) + DH_table(3,1) );
s4 = sin( JointAngle(1,4) + DH_table(4,1) );
s5 = sin( JointAngle(1,5) + DH_table(5,1) );
s6 = sin( JointAngle(1,6) + DH_table(6,1) );

c1 = cos( JointAngle(1,1) + DH_table(1,1) );
c2 = cos( JointAngle(1,2) + DH_table(2,1) );
c3 = cos( JointAngle(1,3) + DH_table(3,1) );
c4 = cos( JointAngle(1,4) + DH_table(4,1) );
c5 = cos( JointAngle(1,5) + DH_table(5,1) );
c6 = cos( JointAngle(1,6) + DH_table(6,1) );

d1 = DH_table(1,2);
d2 = DH_table(2,2);
d3 = DH_table(3,2);
d4 = DH_table(4,2);
d5 = DH_table(5,2);
d6 = DH_table(6,2);

a1 = DH_table(1,3);
a2 = DH_table(2,3);
a3 = DH_table(3,3);
a4 = DH_table(4,3);
a5 = DH_table(5,3);
a6 = DH_table(6,3);


%%  Jacobian 6x6


Jacobian = [ a3*s1*s2*s3 - d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) - d4*(c2*s1*s3 + c3*s1*s2) - a2*c2*s1 - a1*s1 - a3*c2*c3*s1,                                                                                                                                                                                    -c1*(a2*s2 - d4*(c2*c3 - s2*s3) - d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)) + a3*c2*s3 + a3*c3*s2),                                                                                                                                                                       c1*(d4*(c2*c3 - s2*s3) + d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)) - a3*c2*s3 - a3*c3*s2),                                   (d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + d4*(c2*s1*s3 + c3*s1*s2))*(c2*c3 - s2*s3) - (d4*(c2*c3 - s2*s3) + d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)))*(c2*s1*s3 + c3*s1*s2),                                       d6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3))*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)) + d6*s4*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3)))*(c2*s3 + c3*s2),                                                                0  ;
             a1*c1 + d6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d4*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2 - a3*c1*s2*s3 + a3*c1*c2*c3,                                                                                                                                                                                    -s1*(a2*s2 - d4*(c2*c3 - s2*s3) - d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)) + a3*c2*s3 + a3*c3*s2),                                                                                                                                                                       s1*(d4*(c2*c3 - s2*s3) + d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2)) - a3*c2*s3 - a3*c3*s2),                                   (c1*c2*s3 + c1*c3*s2)*(d4*(c2*c3 - s2*s3) + d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2))) - (c2*c3 - s2*s3)*(d6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d4*(c1*c2*s3 + c1*c3*s2)),                                       d6*(c5*(c2*c3 - s2*s3) - c4*s5*(c2*s3 + c3*s2))*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)) - d6*s4*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3)))*(c2*s3 + c3*s2),                                                                0  ;
                                                                                                                                                           0, c1*(d6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d4*(c1*c2*s3 + c1*c3*s2) + a2*c1*c2 - a3*c1*s2*s3 + a3*c1*c2*c3) + s1*(d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + d4*(c2*s1*s3 + c3*s1*s2) + a2*c2*s1 - a3*s1*s2*s3 + a3*c2*c3*s1), s1*(d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + d4*(c2*s1*s3 + c3*s1*s2) - a3*s1*s2*s3 + a3*c2*c3*s1) + c1*(d6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d4*(c1*c2*s3 + c1*c3*s2) - a3*c1*s2*s3 + a3*c1*c2*c3), (d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + d4*(c2*s1*s3 + c3*s1*s2))*(c1*c2*s3 + c1*c3*s2) - (c2*s1*s3 + c3*s1*s2)*(d6*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d4*(c1*c2*s3 + c1*c3*s2)), d6*(c1*c4 + s4*(c2*c3*s1 - s1*s2*s3))*(c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + d6*(c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3)))*(c4*s1 - s4*(c1*c2*c3 - c1*s2*s3)),                                                                0  ;
                                                                                                                                                           0,                                                                                                                                                                                                                                                                                          s1,                                                                                                                                                                                                                                                                    s1,                                                                                                                                                                                                                                 c1*c2*s3 + c1*c3*s2,                                                                                                                                                                                    c4*s1 - s4*(c1*c2*c3 - c1*s2*s3), c5*(c1*c2*s3 + c1*c3*s2) + s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))  ;
                                                                                                                                                           0,                                                                                                                                                                                                                                                                                         -c1,                                                                                                                                                                                                                                                                   -c1,                                                                                                                                                                                                                                 c2*s1*s3 + c3*s1*s2,                                                                                                                                                                                  - c1*c4 - s4*(c2*c3*s1 - s1*s2*s3), c5*(c2*s1*s3 + c3*s1*s2) - s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))  ;
                                                                                                                                                           1,                                                                                                                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                       s2*s3 - c2*c3,                                                                                                                                                                                                 -s4*(c2*s3 + c3*s2),                       c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3)  ];


end