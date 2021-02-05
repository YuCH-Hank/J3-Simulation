function T = calculateT( theta, alpha, a, d )

    RotZ(1,1)=1;
	RotZ(1,2)=0;
	RotZ(1,3)=0;
	RotZ(1,4)=0;

	RotZ(2,1)=0;
	RotZ(2,2)=cos(alpha);
	RotZ(2,3)=-sin(alpha);
	RotZ(2,4)=0;

	RotZ(3,1)=0;
	RotZ(3,2)=sin(alpha);
	RotZ(3,3)=cos(alpha);
	RotZ(3,4)=0;

	RotZ(4,1)=0;
	RotZ(4,2)=0;
	RotZ(4,3)=0;
	RotZ(4,4)=1;

	%-----------------------------------
	TransZ(1,1)=1;
	TransZ(1,2)=0;
	TransZ(1,3)=0;
	TransZ(1,4)=a;

	TransZ(2,1)=0;
	TransZ(2,2)=1;
	TransZ(2,3)=0;
	TransZ(2,4)=0;

	TransZ(3,1)=0;
	TransZ(3,2)=0;
	TransZ(3,3)=1;
	TransZ(3,4)=0;

	TransZ(4,1)=0;
	TransZ(4,2)=0;
	TransZ(4,3)=0;
	TransZ(4,4)=1;

	%-----------------------------------
	TransX(1,1)=1;
	TransX(1,2)=0;
	TransX(1,3)=0;
	TransX(1,4)=0;

	TransX(2,1)=0;
	TransX(2,2)=1;
	TransX(2,3)=0;
	TransX(2,4)=0;

	TransX(3,1)=0;
	TransX(3,2)=0;
	TransX(3,3)=1;
	TransX(3,4)=d;

	TransX(4,1)=0;
	TransX(4,2)=0;
	TransX(4,3)=0;
	TransX(4,4)=1;

    %-----------------------------------
	RotX(1,1)=cos(theta);
	RotX(1,2)=-sin(theta);
	RotX(1,3)=0;
	RotX(1,4)=0;

	RotX(2,1)=sin(theta);
	RotX(2,2)=cos(theta);
	RotX(2,3)=0;
	RotX(2,4)=0;

	RotX(3,1)=0;
	RotX(3,2)=0;
	RotX(3,3)=1;
	RotX(3,4)=0;

	RotX(4,1)=0;
	RotX(4,2)=0;
	RotX(4,3)=0;
	RotX(4,4)=1;

	T=RotX*TransX*TransZ*RotZ;
    
end