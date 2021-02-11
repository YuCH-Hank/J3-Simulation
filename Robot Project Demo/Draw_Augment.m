function Draw_Augment(start, Direction, Width, X1, X2, r, Color, Type)
switch(Type)
    %%
    case ('Triangle')
        % >>>> Draw
        X = [start ;
            start + Direction(1,:) * Width(1);
            start + Direction(1,:) * Width(1) + Direction(2,:) * Width(2);
            start + Direction(2,:) * Width(2);
            start + Direction(3,:) * Width(3);
            start + Direction(1,:) * Width(1) + Direction(3,:) * Width(3);];
        
        face_order = [  1, 2, 3, 4;
                        1, 2, 6, 5;
                        5, 6, 3, 4;];
        for i = 1:3
            BaseX = [X(face_order(i,1),1), X(face_order(i,2),1), X(face_order(i,3),1), X(face_order(i,4),1)];
            BaseY = [X(face_order(i,1),2), X(face_order(i,2),2), X(face_order(i,3),2), X(face_order(i,4),2)];
            BaseZ = [X(face_order(i,1),3), X(face_order(i,2),3), X(face_order(i,3),3), X(face_order(i,4),3)];
            
            patch(BaseX,BaseY,BaseZ,Color)
            hold on;
        end
        
        face_order = [2, 3, 6;
            1, 5, 4;];
        for i = 1:2
            BaseX = [X(face_order(i,1),1), X(face_order(i,2),1), X(face_order(i,3),1)];
            BaseY = [X(face_order(i,1),2), X(face_order(i,2),2), X(face_order(i,3),2)];
            BaseZ = [X(face_order(i,1),3), X(face_order(i,2),3), X(face_order(i,3),3)];
            
            patch(BaseX,BaseY,BaseZ,Color)
            hold on;
        end
        %%
    case('Cube')
        % >>>> Draw
        X = [start ;
            start + Direction(1,:) * Width(1);
            start + Direction(1,:) * Width(1) + Direction(2,:) * Width(2);
            start + Direction(2,:) * Width(2);
            start + Direction(3,:) * Width(3);
            start + Direction(1,:) * Width(1) + Direction(3,:) * Width(3);
            start + Direction(1,:) * Width(1) + Direction(2,:) * Width(2) + Direction(3,:) * Width(3);
            start + Direction(2,:) * Width(2) + Direction(3,:) * Width(3);];
        
        face_order = [1, 2, 3, 4;
            1, 2, 6, 5;
            2, 3, 7, 6;
            5, 6, 7, 8;
            1, 5, 8, 4;
            3, 4, 8, 7];
        for i = 1:6
            BaseX = [X(face_order(i,1),1), X(face_order(i,2),1), X(face_order(i,3),1), X(face_order(i,4),1)];
            BaseY = [X(face_order(i,1),2), X(face_order(i,2),2), X(face_order(i,3),2), X(face_order(i,4),2)];
            BaseZ = [X(face_order(i,1),3), X(face_order(i,2),3), X(face_order(i,3),3), X(face_order(i,4),3)];
            
            patch(BaseX,BaseY,BaseZ,Color)
            hold on;
        end
        %%
    case('Cylinder')
        % Calculating the length of the cylinder
        length_cyl = norm(X2-X1);
        
        % Creating a circle in the YZ plane
        t  = linspace(0,2*pi,15)';
        x2 = r*cos(t);
        x3 = r*sin(t);
        
        % Creating the points in the X-Direction
        x1 = [0 length_cyl];
        
        % Creating (Extruding) the cylinder points in the X-Directions
        xx1 = repmat(x1,length(x2),1);
        xx2 = repmat(x2,1,2);
        xx3 = repmat(x3,1,2);
        
        % Drawing two filled cirlces to close the cylinder
        hold on
        EndPlate1 = fill3(xx1(:,1),xx2(:,1),xx3(:,1),'r');
        EndPlate2 = fill3(xx1(:,2),xx2(:,2),xx3(:,2),'r');
        
        % Plotting the cylinder along the X-Direction with required length starting
        % from Origin
        Cylinder = mesh(xx1,xx2,xx3);
        
        % Defining Unit vector along the X-direction
        unit_Vx = [1 0 0];
        
        % Calulating the angle between the x direction and the required direction
        % of cylinder through dot product
        angle_X1X2=acos( dot( unit_Vx,(X2-X1) )/( norm(unit_Vx)*norm(X2-X1)) )*180/pi;
        
        % Finding the axis of rotation (single rotation) to roate the cylinder in
        % X-direction to the required arbitrary direction through cross product
        axis_rot=cross([1 0 0],(X2-X1) );
        
        % Rotating the plotted cylinder and the end plate circles to the required
        % angles
        if angle_X1X2~=0 % Rotation is not needed if required direction is along X
            rotate(Cylinder,axis_rot,angle_X1X2,[0 0 0])
            rotate(EndPlate1,axis_rot,angle_X1X2,[0 0 0])
            rotate(EndPlate2,axis_rot,angle_X1X2,[0 0 0])
        end
        
        % Till now cylinder has only been aligned with the required direction, but
        % position starts from the origin. so it will now be shifted to the right
        % position
        set(EndPlate1,'XData',get(EndPlate1,'XData')+X1(1))
        set(EndPlate1,'YData',get(EndPlate1,'YData')+X1(2))
        set(EndPlate1,'ZData',get(EndPlate1,'ZData')+X1(3))
        
        set(EndPlate2,'XData',get(EndPlate2,'XData')+X1(1))
        set(EndPlate2,'YData',get(EndPlate2,'YData')+X1(2))
        set(EndPlate2,'ZData',get(EndPlate2,'ZData')+X1(3))
        
        set(Cylinder,'XData',get(Cylinder,'XData')+X1(1))
        set(Cylinder,'YData',get(Cylinder,'YData')+X1(2))
        set(Cylinder,'ZData',get(Cylinder,'ZData')+X1(3))
        
        % Setting the color to the cylinder and the end plates
        set(Cylinder,'FaceColor',Color)
        set([EndPlate1 EndPlate2],'FaceColor',Color)
        set(Cylinder,'EdgeAlpha',0)
end
end