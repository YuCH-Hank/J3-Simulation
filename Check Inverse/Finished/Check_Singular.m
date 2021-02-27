clc, close all;

if (false)
    clear;
    S_120 = load('Singular Point_120.txt');
    Singular_120.x = S_120(:,1); Singular_120.y = S_120(:,2); Singular_120.z = S_120(:,3); Singular_120.all = S_120; 
    Singular_120.center = [ sum(Singular_120.x)/length(S_120(:,1)), sum(Singular_120.y)/length(S_120(:,1)), sum(Singular_120.z)/length(S_120(:,1))];
    Singular_120.num = numel(unique(Singular_120.z));
    clear S_120;
    
    S = load('Singular Point.txt');
    Singular.x = S(:,1); Singular.y = S(:,2); Singular.z = S(:,3); Singular.all = S; 
    Singular.center = [ sum(Singular.x)/length(S(:,1)), sum(Singular.y)/length(S(:,1)), sum(Singular.z)/length(S(:,1))];
    Singular.num = numel(unique(Singular.z));
    clear S;
end
%% Plot
Point = Singular_120;
plot3(Point.x, Point.y, Point.z, '.', Point.center(1), Point.center(2), Point.center(3), 'ro');
axis([-40, 80, -40, 40, 2, 120]);
axis equal;
hold on;
line([Point.center(1), Point.center(1)], [Point.center(2), Point.center(2)], [-2, 120], 'Color', 'Red' );
hold off;

%% Calculate Radius
z = linspace(2, 120, Point.num);
len = ones(Point.num, 2) * 500;

for i = 1 : Point.num
    t = find( abs(Point.z - z(i)) < 0.00001);
    p.x = Point.x(t);    p.y = Point.y(t);    p.z = Point.z(t);
    p_cen_x = ones(length(t), 1) * Point.center(1);
    p_cen_y = ones(length(t), 1) * Point.center(2);
    len_ = ((p.x - p_cen_x) .^2 + (p.y - p_cen_y) .^2 ).^(1/2);
    
    if (z(i) > 50 && z(i) < 60)
        len_ = sort([len_;20]);
        k = find(len_ == 20);
        
        if (k == 1)
            len(i,1) = len_(k+1);
            len(i,2) = 0;
        else
            len(i,1) = len_(k + 1);
            len(i,2) = len_(k - 1);
        end
    else
        len(i,1) = min(len_);
        len(i,2) = 0;
    end
    
    i = i + 1;
end

%%
C = [z',len];
Generate_txt([0, Point.center(1), Point.center(2);C]','Limitation');

%%
if (false)
    for  i  = 1 : Point.num
        t = find( round(Point.z * 10) == round(z(i) * 10));
        p.x = Point.x(t);    p.y = Point.y(t);    p.z = Point.z(t);
        
        theta = linspace(0,2*pi,100);
        plot(Point.center(1) + len(i,1) * cos(theta), Point.center(2) + len(i,1) * sin(theta),'-')
        hold on;
        axis equal
        
        theta = linspace(0,2*pi,100);
        plot(Point.center(1) + len(i,2) * cos(theta), Point.center(2) + len(i,2) * sin(theta),'-')
        hold on;
        axis equal
        
        plot(p.x, p.y, '.')
        title(sprintf('Z = %d', z(i)));
        hold off;
        M = input('Enter'' to continue 1 to stop','s');
        if (M == '1')
            break;
        end
    end
end