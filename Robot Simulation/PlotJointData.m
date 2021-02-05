function Record = PlotJointData(Record, Time, time, PLOT)

J_Record = Record.Joint.JointRecord;
order    = PLOT.record.recorder;

%% >>>> ITRI Parameter

% >>>> ITRI_parameter
[ ITRI_parameter , DH_table , SamplingTime ] = ITRI_Parameter;
DOF = size( DH_table , 1 ) ;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

% >>>> 
for i = 1:length(time)
    seperate(i) = sum(time(1:i));
end
seperate = seperate .*  SamplingTime;
%% >>>> Joint
figure( 'name','PoseRecord' );

for i = 1 : 6
    subplot(2,3,i)
    plot(  Time ,   J_Record(:,i));
    title(sprintf('Joint %d', i));
    hold on;
    plot ( Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Pos(i,2) , 'r--', ...
           Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Pos(i,1) , 'r--');
    hold on;
    plot ( Time, Record.Joint.JointCmd(i,:) , 'k-');
    axis ([0, max(Time), ITRI_Limitation.Joint.Pos(i,2), ITRI_Limitation.Joint.Pos(i,1)]);
    xlabel('Time(s)');
    ylabel('Position(rad)');
    
    % >>>> check limitation
    if min (J_Record(:,i)) < ITRI_Limitation.Joint.Pos(i,2)
        mini = min (J_Record(:,i)) * 1.1;
    else
        mini = ITRI_Limitation.Joint.Pos(i,2) * 1.1;
    end
    if max (J_Record(:,i)) > ITRI_Limitation.Joint.Pos(i,1)
        maxi = max (J_Record(:,i)) * 1.1;
    else
        maxi = ITRI_Limitation.Joint.Pos(i,1) * 1.1;
    end
    
    % >>>> axis limitation
    axis ([0, max(Time), mini , maxi]);
    
    if (PLOT.record.time)
        % >>>> plot time
        for t = 1:length(seperate)-1
            plot(ones(1, 100) .* seperate(t), linspace(mini, maxi, 100), 'b--');
            hold on;
        end
    end
      
    hold off;
end
if(PLOT.record.Save_Plot)
    str = datestr(now,30);
    str = ['PLOT/',str ,'_Pose.jpg'];
    saveas(gcf, str);
end

%% Velocity record
if order > 0
    J_d = derivate(J_Record, SamplingTime, order);
    
    if(PLOT.record.low_pass)
        J_d = lowpass(J_d,PLOT.record.low_pass_f,SamplingTime^-1);
    end
    Record.Performance.Derivate = J_d;
    figure( 'name','VelocityRecord' );
    
    for i = 1 : 6
        subplot(2,3,i)
        plot(  Time ,   J_d(:,i + 6));
        title(sprintf('Joint %d', i));
        hold on;
        plot ( Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Vel(i) , 'r--', ...
               Time, ones(length(Time), 1) * -ITRI_Limitation.Joint.Vel(i) , 'r--');
        hold on;
        plot ( Time, Record.Joint.VelCmd(i,:) , 'k-');
        % >>>> check limitation
        if min (J_d(:, i + 6)) < - ITRI_Limitation.Joint.Vel(i)
            mini = min (J_d(:, i + 6))  * 1.1;
        else
            mini = - ITRI_Limitation.Joint.Vel(i)  * 1.1;
        end
        if max (J_d(:, i + 6)) > ITRI_Limitation.Joint.Vel(i)
            maxi = max (J_d(:, i + 6))  * 1.1;
        else
            maxi = ITRI_Limitation.Joint.Vel(i) * 1.1;
        end
        
        % >>>> axis limitation
        axis ([0, max(Time), mini , maxi]);
        
        if (PLOT.record.time)
            for t = 1:length(seperate)-1
                plot(ones(1, 100) .* seperate(t), linspace(mini, maxi, 100), 'b--');
                hold on;
            end
        end
        

        xlabel('Time(s)');
        ylabel('Velocity(rad/s)');
        title(sprintf('Joint %d', i));
        hold off;
    end
    if(PLOT.record.Save_Plot)
        str = datestr(now,30);
        str = ['PLOT/', str ,'_Vec.jpg'];
        saveas(gcf, str);
    end
end

%% Accleration record
if order > 1    
    figure( 'name','AcclerationRecord' );
    
    for i = 1 : 6
        subplot(2,3,i)
        plot(  Time ,   J_d(:,i + 12));
        title(sprintf('Joint %d', i));
        hold on;
        plot ( Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Acc(i) , 'r--', ...
               Time, ones(length(Time), 1) * -ITRI_Limitation.Joint.Acc(i) , 'r--');
        
        % >>>> check limitation
        if min (J_d(:, i + 12)) < - ITRI_Limitation.Joint.Acc(i)
            mini = min (J_d(:, i + 12)) * 1.1;
        else
            mini = - ITRI_Limitation.Joint.Acc(i) * 1.1;
        end
        if max (J_d(:, i + 12)) > ITRI_Limitation.Joint.Acc(i)
            maxi = max (J_d(:, i + 12)) * 1.1;
        else
            maxi = ITRI_Limitation.Joint.Acc(i) * 1.1;
        end
        
        % >>>> axis limitation
        axis ([0, max(Time), mini , maxi]);
        
        if (PLOT.record.time)
            % >>>> plot time
            for t = 1:length(seperate)-1
                plot(ones(1, 100) .* seperate(t), linspace(mini, maxi, 100), 'b--');
                hold on;
            end
        end
        
               
        xlabel('Time(s)');
        ylabel('Accleration(rad/s^2)');
        title(sprintf('Joint %d', i));
        hold off;
    end
    if(PLOT.record.Save_Plot)
        str = datestr(now,30);
        str = ['PLOT/', str ,'_Acc.jpg'];
        saveas(gcf, str);
    end
end

%% Jerk record
if order > 2
    figure( 'name','JerkRecord' );
    
    for i = 1 : 6
        subplot(2,3,i)
        plot(  Time ,   J_d(:,i + 18));
        title(sprintf('Joint %d', i));
        hold on;
        plot ( Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Jerk(i) , 'r--', ...
               Time, ones(length(Time), 1) * -ITRI_Limitation.Joint.Jerk(i) , 'r--');
        
        % >>>> check limitation
        if min (J_d(:, i + 18)) < - ITRI_Limitation.Joint.Jerk(i)
            mini = min (J_d(:, i + 18)) * 1.1;
        else
            mini = - ITRI_Limitation.Joint.Jerk(i) * 1.1;
        end
        if max (J_d(:, i + 18)) > ITRI_Limitation.Joint.Jerk(i)
            maxi = max (J_d(:, i + 18)) * 1.1;
        else
            maxi = ITRI_Limitation.Joint.Jerk(i) * 1.1;
        end
        
        % >>>> axis limitation
        axis ([0, max(Time), mini , maxi]);
        
        if (PLOT.record.time)
            % >>>> plot time
            for t = 1:length(seperate)-1
                plot(ones(1, 100) .* seperate(t), linspace(mini, maxi, 100), 'b--');
                hold on;
            end
        end
                
        xlabel('Time(s)');
        ylabel('Jerk(rad/s^3)');
        title(sprintf('Joint %d', i));
        hold off;
    end
    if(PLOT.record.Save_Plot)
        str = datestr(now,30);
        str = ['PLOT/', str ,'_Jerk.jpg'];
        saveas(gcf, str);
    end
end
%% Snap record
if order > 3
    figure( 'name','SnapRecord' );
    
    for i = 1 : 6
        subplot(2,3,i)
        plot(  Time ,   J_d(:,i + 24));
        title(sprintf('Joint %d', i));
        hold on;
        plot ( Time, ones(length(Time), 1) * ITRI_Limitation.Joint.Jerk(i)*10 , 'r--', ...
               Time, ones(length(Time), 1) * -ITRI_Limitation.Joint.Jerk(i)*10 , 'r--');
        
        % >>>> check limitation
        if min (J_d(:, i + 24)) < - ITRI_Limitation.Joint.Jerk(i)*10
            mini = min (J_d(:, i + 24)) * 1.1;
        else
            mini = - ITRI_Limitation.Joint.Jerk(i) * 1.1 *10;
        end
        if max (J_d(:, i + 24)) > ITRI_Limitation.Joint.Jerk(i) *10
            maxi = max (J_d(:, i + 24)) * 1.1;
        else
            maxi = ITRI_Limitation.Joint.Jerk(i) * 1.1 *10;
        end
        
        % >>>> axis limitation
        axis ([0, max(Time), mini , maxi]);
        
        if (PLOT.record.time)
            % >>>> plot time
            for t = 1:length(seperate)-1
                plot(ones(1, 100) .* seperate(t), linspace(mini, maxi, 100), 'b--');
                hold on;
            end
        end
        
        xlabel('Time(s)');
        ylabel('Snap(rad/s^4)');
        title(sprintf('Joint %d', i));
        hold off;
    end
    
    if(PLOT.record.Save_Plot)
        str = datestr(now,30);
        str = ['PLOT/', str ,'_Snap.jpg'];
        saveas(gcf, str);
    end
end
end