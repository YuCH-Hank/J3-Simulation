clc,clear
% Cartesian Demo
Pos = [ -40 + 120*rand(1,1),...
        -40 + 80*rand(1,1),...
        2 + 118*rand(1,1)];

sing = Cartesian_Limitation(Pos, 'Limitation');
if (sing)
    clc;
    fprintf('>>>> %d, %d, %d,\n Singular\n', Pos(1), Pos(2), Pos(3));
else
    clc;
    fprintf('>>>> %d, %d, %d,\n Normal\n', Pos(1), Pos(2), Pos(3));
end

S = load('Singular Point.txt');
Singular.x = S(:,1); Singular.y = S(:,2); Singular.z = S(:,3); Singular.all = S;
Singular.center = [ sum(Singular.x)/length(S(:,1)), sum(Singular.y)/length(S(:,1)), sum(Singular.z)/length(S(:,1))];
Singular.num = numel(unique(Singular.z));
clear S;

Point = Singular;
if (sing)
    plot3(Point.x, Point.y, Point.z, '.', Pos(1), Pos(2), Pos(3), 'ro');
    
else
    plot3(Point.x, Point.y, Point.z, '.', Pos(1), Pos(2), Pos(3), 'bo');
    
end
view(180,0);
set(gcf, 'Position',  [800, 0, 800,800])