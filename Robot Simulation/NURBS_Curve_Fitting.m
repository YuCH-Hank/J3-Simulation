clc,clear,close all;
%% 2020.10.18 NURBS
Data_Point = [  0 2   3 3.2 5 10 11 13 15 16 17 20 ;
               -1 2 2.5   3 6  7  8 10 12 15 16 18;
                0 1   3   5  6 8  3  1  2  3  4  5];
samp = 0.001;

n_max = 7;
p_max = n_max;
n = linspace(2,n_max,n_max-1);
p = linspace(2,p_max,p_max-1);

num = 1;
for j = 1:p_max
    for i = 1 : n_max
        try
            Parameter(num) = NURBS_curve_fitting_function (Data_Point , n(i) , p(j) , samp);
            hold on;
            Error(num) = Parameter(num).Error;
        catch
            num = num - 1;
        end
        num = num + 1;
    end
end

[Min,num] = min(Error);
Parameter(num)
fprintf('Minimum Error = %d , n = %d, p = %d', Parameter(num).Error, Parameter(num).n, Parameter(num).p);
figure (2)
plot(Data_Point(1,:),Data_Point(2,:),'ro',Parameter(num).Curve(:,1),Parameter(num).Curve(:,2));
axis([0 20 -2 18]);
figure (3)
plot(Parameter(num).Control_Point(:,1),Parameter(num).Control_Point(:,2),'b+');
axis([0 20 -2 18]);
figure (4)
plot(Data_Point(1,:),Data_Point(2,:),'ro');
axis([0 20 -2 18]);