clc,clear,close all;
%% %%%%%%%%%%%%% PLOT DATA %%%%%%%%%%%%%%%%%
% Date:2021/01/26
%=================================================

Velocity = struct(  'O',     [],...
                    'T',    [],...
                    'M',     []);
                
Position = struct(  'O',     [],...
                    'T',    [],...
                    'M',     []);

% >>>> Load Data
Velocity.O  = load('Record_Original_Velocity');
Velocity.T  = load('Record_Time_sync_Velocity');
Velocity.M  = load('Record_Minimize_Jerk_Velocity');

Position.O  = load('Record_Original_Position');
Position.T  = load('Record_Time_sync_Position');
Position.M  = load('Record_Minimize_Jerk_Position');
%%
% >>>> Norm performance Velocity Controller
% >>>> Velocity
figure(1)
NORM_V = [  Velocity.O.Record.Performance.Velocity, ...
            Velocity.T.Record.Performance.Velocity, ...
            Velocity.M.Record.Performance.Velocity];
b = bar(NORM_V);
legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
title('Velocity Controller Velocity error');
xlabel('Joint');
ylabel('Norm velocity error')
set(gcf, 'Position',  [0, 0, 8000, 12000])

plot_data(b, 1);
plot_data(b, 2);
plot_data(b, 3);

file_name = 'Velocity_Controller_Velocity_error';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

% >>>> Position
figure(2)
NORM_P = [  Velocity.O.Record.Performance.Position, ...
            Velocity.T.Record.Performance.Position, ...
            Velocity.M.Record.Performance.Position];
b = bar(NORM_P);
title('Velocity Controller Position error');
legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
ylabel('Norm position error');
xlabel('Joint');
set(gcf, 'Position',  [0, 0, 8000, 12000])

plot_data(b, 1);
plot_data(b, 2);
plot_data(b, 3);


file_name = 'Velocity_Controller_Position_error';
file_name = [file_name, '.jpg'];
saveas(gcf, file_name);
%%
% >>>> Norm performance Position Controller
% >>>> Velocity
figure(3)
NORM_V = [  Position.O.Record.Performance.Velocity, ...
            Position.T.Record.Performance.Velocity, ...
            Position.M.Record.Performance.Velocity];
b = bar(NORM_V);
title('Position Controller Velocity error');
legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
xlabel('Joint');
ylabel('Norm velocity error')
set(gcf, 'Position',  [0, 0, 8000, 12000])

plot_data(b, 1);
plot_data(b, 2);
plot_data(b, 3);

file_name = 'Position_Controller_Velocity_error';
file_name = [file_name, '.jpg'];
saveas(gcf, file_name);

% >>>> Position
figure(4)
NORM_P = [  Position.O.Record.Performance.Position, ...
            Position.T.Record.Performance.Position, ...
            Position.M.Record.Performance.Position];
b = bar(NORM_P);
title('Position Controller Position error');
legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
ylabel('Norm position error');
xlabel('Joint');
set(gcf, 'Position',  [0, 0, 8000, 12000])

plot_data(b, 1);
plot_data(b, 2);
plot_data(b, 3);

file_name = 'Position_Controller_Position_error';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);
%% 

SamplingTime = 0.001;
figure('name','Velocity Controller Position');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Velocity.O.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.O.Record.Joint.JointRecord(:,i),...
         get_linespace(Velocity.T.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.T.Record.Joint.JointRecord(:,i),...
         get_linespace(Velocity.M.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.M.Record.Joint.JointRecord(:,i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Position(rad)');
    grid on;
end

set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Velocity_Controller_Position';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Velocity Controller Velocity');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Velocity.O.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.O.Record.Performance.Derivate(:,6+i),...
         get_linespace(Velocity.T.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.T.Record.Performance.Derivate(:,6+i),...
         get_linespace(Velocity.M.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.M.Record.Performance.Derivate(:,6+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Velocity(rad/s)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Velocity_Controller_Velocity';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Velocity Controller Accleration');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Velocity.O.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.O.Record.Performance.Derivate(:,12+i),...
         get_linespace(Velocity.T.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.T.Record.Performance.Derivate(:,12+i),...
         get_linespace(Velocity.M.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.M.Record.Performance.Derivate(:,12+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Accleration(rad/s^2)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Velocity_Controller_Accleration';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Velocity Controller Jerk');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Velocity.O.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.O.Record.Performance.Derivate(:,18+i),...
         get_linespace(Velocity.T.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.T.Record.Performance.Derivate(:,18+i),...
         get_linespace(Velocity.M.Record.Joint.JointRecord(:,i),SamplingTime), Velocity.M.Record.Performance.Derivate(:,18+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Jerk(rad/s^3)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Velocity_Controller_Jerk';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

%% 
SamplingTime = 0.001;
figure('name','Position Controller Position');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Position.O.Record.Joint.JointRecord(:,i),SamplingTime), Position.O.Record.Joint.JointRecord(:,i),...
         get_linespace(Position.T.Record.Joint.JointRecord(:,i),SamplingTime), Position.T.Record.Joint.JointRecord(:,i),...
         get_linespace(Position.M.Record.Joint.JointRecord(:,i),SamplingTime), Position.M.Record.Joint.JointRecord(:,i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Position(rad)');
    grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Position_Controller_Position';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Position Controller Velocity');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Position.O.Record.Joint.JointRecord(:,i),SamplingTime), Position.O.Record.Performance.Derivate(:,6+i),...
         get_linespace(Position.T.Record.Joint.JointRecord(:,i),SamplingTime), Position.T.Record.Performance.Derivate(:,6+i),...
         get_linespace(Position.M.Record.Joint.JointRecord(:,i),SamplingTime), Position.M.Record.Performance.Derivate(:,6+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Velocity(rad/s)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Position_Controller_Velocity';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Position Controller Accleration');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Position.O.Record.Joint.JointRecord(:,i),SamplingTime), Position.O.Record.Performance.Derivate(:,12+i),...
         get_linespace(Position.T.Record.Joint.JointRecord(:,i),SamplingTime), Position.T.Record.Performance.Derivate(:,12+i),...
         get_linespace(Position.M.Record.Joint.JointRecord(:,i),SamplingTime), Position.M.Record.Performance.Derivate(:,12+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Accleration(rad/s^2)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Position_Controller_Accleration';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);

figure('name','Position Controller Jerk');
for i = 1:6
    subplot(2,3,i)
    plot(get_linespace(Position.O.Record.Joint.JointRecord(:,i),SamplingTime), Position.O.Record.Performance.Derivate(:,18+i),...
         get_linespace(Position.T.Record.Joint.JointRecord(:,i),SamplingTime), Position.T.Record.Performance.Derivate(:,18+i),...
         get_linespace(Position.M.Record.Joint.JointRecord(:,i),SamplingTime), Position.M.Record.Performance.Derivate(:,18+i))
     legend('Original', 'Time sync', 'Minimize Jerk','Location' , 'southeast');
    title(sprintf('J%d',i ));
    xlabel('time(s)');
    ylabel('Jerk(rad/s^3)');
        grid on;
end
set(gcf, 'Position',  [0, 0, 8000, 12000])
file_name = 'Position_Controller_Jerk';
file_name = [ file_name, '.jpg'];
saveas(gcf, file_name);
close all
%% Functions
function plot_data(bar, i)
    xtips = bar(i).XEndPoints;
    ytips = bar(i).YEndPoints;
    labels = string(round(bar(i).YData*10)/10);
    text(xtips,ytips,labels,'HorizontalAlignment','center',...
        'VerticalAlignment','bottom')
end
function n = get_linespace(data, SamplingTime)
    len = length(data);
    n = linspace(1,len-1,len) .* SamplingTime;
end