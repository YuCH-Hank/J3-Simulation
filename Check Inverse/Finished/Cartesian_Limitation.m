function Singular = Cartesian_Limitation(Pos, path)
x = Pos(1); y = Pos(2); z = Pos(3);

Limitation = load([path, '.txt']);
center_x = Limitation(1,2);
center_y = Limitation(1,3);

Limitation = Limitation(2:end,:);

% Find z
Z = sort([z; Limitation(1:end,1)]);

k = find(Z == z);

z_lower = k(1) - 1; z_upper = k(1);

r = sqrt((x - center_x) ^2 + (y - center_y)^2);

% for z_lower
if (r < Limitation(z_lower, 2) && r > Limitation(z_lower, 3) && r < Limitation(z_upper, 2) && r > Limitation(z_upper, 3))
    Singular = false;
    
else
    Singular = true;
    
end


end