clc,clear,close all
% >>>> ITRI Parameter
[~, DH_table, ~] = ITRI_Parameter;

%%
num = 1000;

Err_index = cell( num, 1);
singular_index = cell( num, 1);
Diff_index = cell( num, 1);

parfor  i = 1: num
    i
    NowEulerAngle = [-131.071000000000,-31.4922903017591,-33.1470000000000];
    x = linspace(-40,80,num);
    y = linspace(-40,40,num);
    z = linspace(2,120,num);
    j = 1; k = 1;
    e = [];    si = [];    diff = [];
    while( j <= num )
        while( k <= num )
            try
                [ InverseJointAngle_Final , SingularFlag2 ] = InverseKinemetics( NowEulerAngle , [x(i), y(j), z(k)] , DH_table ) ;
                
                if (SingularFlag2)
                    % fprintf(' x = %.2d, y = %.2d, z = %.2d \tSingular \n', i, j, k);
                    si = [si; j, k];
                else
                    t = 1;
                    while(t < 9)
                        [ Info  ,  NowEulerAngle , Ini ] = ForwardKinemetics( DH_table , InverseJointAngle_Final(t,:)');
                        
                        % Check Forward
                        if ( sqrt(sum(([x(i),y(j),z(k)] - Ini).^2)) > 1)
                            %                         fprintf('x = %d, y = %d, z = %d \n', x(i), y(j), z(k));
                            diff  = [diff; j, k];
                            fprintf('x = %d, y = %d, z = %d Forward \n', x(i), y(j), z(k));
                            break;
                        end
                        
                        % Check Difference
                        for h = t + 1 : 8
                            I = InverseJointAngle_Final(t,:)';
                            if ( sqrt(sum((I - InverseJointAngle_Final(h,:)').^2)) == 0)
                                %                         fprintf('x = %d, y = %d, z = %d \n', x(i), y(j), z(k));
                                diff = [diff; j, k];
                                fprintf('x = %d, y = %d, z = %d Same \n', x(i), y(j), z(k));
                                break;
                            end
                        end
                        t = t + 1;
                    end
                end
            catch err
                err
                e = [e; j, k];
            end
            k = k + 1;
        end
        j = j + 1; k = 1;
    end
        
    Diff_index{i} = diff;
    singular_index{i} =  si;
    Err_index{i} = e;
    j = 1; k = 1;
end

%%
Diff_index = check(Diff_index, num);
singular_index = check(singular_index, num);
Err_index = check(Err_index, num);

if (~isempty(Diff_index))
    D = Transform(Diff_index, num);
else
    fprintf('No Points are different\n');
    clear Diff_index
end
if (~isempty(singular_index))
    Singular_Point = Transform(singular_index, num);
end
if (~isempty(Err_index))
    E = Transform(Err_index, num);
else
    fprintf('No Points has errors\n');
   clear Err_index
end
clear num DH_table

Generate_txt( Singular_Point', 'Data\Singular Point');

%%
Singular_Point = load('Data\Singular Point.txt');

plot3(Singular_Point(:,1),Singular_Point(:,2),Singular_Point(:,3),'.');
axis equal;
axis([-40, 80, -40, 40, 2, 120]);
grid on;
xlabel('x-axis');
ylabel('y-axis');
zlabel('z-axis');

function Output = check(V, num)
O = []; Output = []; 
for i = 1 : num
    C = [];
    if (~isempty(V{i}))
        C(:,:) =  V{i}(:,:);
        C = [ones(length(V{i}(:,1)), 1) * i, C];
        O = [O; C];
    else
        C =[i, 0, 0];
        O = [O; C];
    end
end
for i = 1 : length(O(:,1))
    if (O(i,2) ~= 0)
       Output = [Output; O(i,:)]; 
    end
end
end

function Output = Transform(V, num)
    x = linspace(-40,80,num);
    y = linspace(-40,40,num);
    z = linspace(2,120,num);
    Output = [];
    for i = 1:length(V(:,1))
       Output = [Output; x(V(i,1)), y(V(i,2)), z(V(i,3))]; 
    end
end