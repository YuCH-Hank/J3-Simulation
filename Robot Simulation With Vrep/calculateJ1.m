function [ J1 , J1p ] = calculateJ1( Ex,Ey,Ez) 

J1  = atan2(Ey,Ex);
J1p = atan2(Ey,Ex) + pi;
% 增加一個機制 若超過180度 自動變負號
J1 = over180_rad(J1);
J1p = over180_rad(J1p);

end