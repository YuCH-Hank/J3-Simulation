function [ J1 , J1p ] = calculateJ1( Ex,Ey,Ez) 

J1  = atan2(Ey,Ex);
J1p = atan2(Ey,Ex) + pi;
% �W�[�@�Ӿ��� �Y�W�L180�� �۰��ܭt��
J1 = over180_rad(J1);
J1p = over180_rad(J1p);

end