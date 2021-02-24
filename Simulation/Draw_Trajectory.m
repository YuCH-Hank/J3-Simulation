function DrawTrajectory( Trajectory  )

 	Xside = Trajectory(1:size(Trajectory),1);
    Yside = Trajectory(1:size(Trajectory),2);
    Zside = Trajectory(1:size(Trajectory),3);
    
    plot3( Xside , Yside , Zside , 'k' );
    hold on;
        
end