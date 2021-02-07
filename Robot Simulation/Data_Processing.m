function Output = Data_Processing(Input, save_, path, low_pass_)
%% >>>> low_pass
if (low_pass_)
    Input = lowpass(Input,100,1000);
end


%% >>>>>>> Find Segment
Zero = find(Input(:,2) > -5);

segment = [];
for i = 2 : length(Zero)
    if ((Zero(i) - Zero(i-1)) ~= 1)
        segment = [segment; Zero(i-1), Zero(i)-1];
    end
end

number = zeros(1,length(segment));
for i = 2 : length(segment)
   for j = segment(i,2) : -1 : segment(i , 1) + 20
      if (abs(Input(j, 2) - Input(j + 1, 2))  > 5)
          number(i) = j;
      end
   end
end

for i = 2 : length(number)
   if (number(i)~=0)
       segment(i , 2) = number(i);
   end
end

%% >>>>>>> Command Generation
Command = struct (  'Joint',    [],...
                    'Converge', [],...
                    'segment',  segment',...
                    'Usable_C', [],...
                    'Total_C',  1);
                
for i = 1 : length(segment)
    Command.Joint{i} = Input(Command.segment(1,i):Command.segment(2,i) , :) / 180 * pi();
    if (Command.segment(2,i) - Command.segment(1,i)) > 400
        Command.Converge = [Command.Converge, false];
    else
        Command.Converge = [Command.Converge, true];
    end
end

for i = 1 : length(Command.Joint)
    if (Command.Converge(i))
        Command.Usable_C{Command.Total_C} = Command.Joint{i};
        Command.Total_C = Command.Total_C + 1;
    end
end
Command.Total_C = Command.Total_C - 1;

fprintf('Total Usable Command number %d\n', Command.Total_C);

%% Figure
figure('name', 'All Command');
for i = 1 : length(Command.Usable_C)
    for j = 1:6
       subplot(2,3,j);
       plot(Command.Usable_C{i}(:,j));
       title(sprintf('J%d',j))
       ylabel('Rad');
       hold on;
    end
end

Output = Command;
if (save_)
    save( path);
end
end