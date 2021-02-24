function Draw_Obstacle( Center , Radius )

    [x,y,z]  = ellipsoid( Center(1,1) , Center(1,2) , Center(1,3) , Radius , Radius , Radius );
    surf(x,y,z)

    hold on;
    
end