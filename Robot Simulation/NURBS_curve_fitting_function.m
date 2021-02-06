function Parameter = NURBS_curve_fitting_function (Data_Point , n , p , samp)

k = p + 1;
m = size(Data_Point,2) - 1;

%% u 
u = zeros(1,m + 1);

sum = 0;
for i = 2 : m + 1
    sum = sum + (abs(Data_Point(1,i) - Data_Point(1,i-1)))^(1/2) + (abs(Data_Point(2,i) - Data_Point(2,i-1)))^(1/2);
end

for i = 2:m+1
    u(1,i) = u(1,i-1) + ((abs(Data_Point(1,i) - Data_Point(1,i-1)))^(1/2) + (abs(Data_Point(2,i) - Data_Point(2,i-1)))^(1/2)) / sum;
end

%% knot vector
v = zeros(1,n-p);

for i = 1 : n-p
    sum = zeros(1,1);
    for j = 2 : m - n + p + i
        sum = sum + u(1,j);
    end
    v(1,i) = 1/(m-n+p) * sum;
end

knot_vector = [zeros(1,1+p) v ones(1,1+p)];

%% Bending Function

N = Bending_Function(knot_vector,samp,p);

NN = zeros(m+1,n+1);

for j = 1:n+1
    for i = 1 : m+1
        NN(i,j) = N(j,k, round(u(i)/samp + 1));
    end
end

Control_Point = (pinv(NN)*Data_Point');

for t = 0 : samp : max(knot_vector)
    sum = 0;
    for i = 1 : size(Control_Point,1)
        sum = sum + N(i,k, round(t/samp + 1)) * Control_Point(i,:);
    end
    C(round(t/samp + 1),:) = sum;
end

figure('name', 'NURBS Fitting')
plot3(Data_Point(1,:),Data_Point(2,:),Data_Point(3,:),'ro',C(:,1),C(:,2),C(:,3));
grid on;

%% Error
Error = 0;
for i = 1 : 1 + m
    Error = Error + (Data_Point(1,i) - C(round(u(i)/samp + 1),1))^2 + (Data_Point(2,i) - C(round(u(i)/samp + 1),2))^2;
end


Parameter = struct('n',n,'p',p,'Error',Error,'Knot_vector',knot_vector,'Curve',C,'Control_Point',Control_Point);
end