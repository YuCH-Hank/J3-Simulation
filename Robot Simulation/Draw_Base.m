%%  畫圖 畫物件 輸入: center(x, y), Width(a, b, c), Color
function Draw_Base()
% >>>> Color
Gray        = [ 0.6627 , 0.6627 , 0.6627 ];
Black       = [ 0      , 0      , 0];
Dark_Gray   = [ 0.5    , 0.5    , 0.5];

% >>>> Object
% >>>> 長方體
% >>>> x, y, z
Width_rect  = [ 23,    23,    2;
                30,    30,    2;
               115,    80, 76.5;
                 13,    5,   18;];
           
center_rect = [ 0,      0,          -24;
                0,      0,          -26;
                30, -5.25,   -76.5/2-27;
               -12,    -1,            0;];
    
Color_rect  = [ Gray;
                Black;
                Gray;
                Dark_Gray];
% >>>> 圓柱體
start_point = [0, 0, 0];

end_point   = [0, 0, -23];

r_cyl       = [23/2, 23/2];

Color_cyl   = [Gray];

% >>>> 長方體
for i = 1 : length(Color_rect(:,1))
    Width   = Width_rect(i,:);
    center  = center_rect(i,:);
    Color   = Color_rect(i,:);
    
    
    X1 =  Width(1)/2 + center(1);
    X2 = -Width(1)/2 + center(1);
    Y1 =  Width(2)/2 + center(2);
    Y2 = -Width(2)/2 + center(2);
    Z1 =  Width(3)/2 + center(3);
    Z2 = -Width(3)/2 + center(3);

    %% ------ 基底 ------------------------------- 共有6個面
    % face 1
    BaseX = [X1 X1 X2 X2];
    BaseY = [Y1 Y2 Y2 Y1];
    BaseZ = [Z1 Z1 Z1 Z1];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on;
    % face 2
    BaseX = [X1 X1 X1  X1];
    BaseY = [Y1 Y2 Y2  Y1];
    BaseZ = [Z1 Z1 Z2  Z2];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on;
    % face 3
    BaseX = [X1 X2 X2  X1];
    BaseY = [Y1 Y1 Y1 Y1];
    BaseZ = [Z1 Z1 Z2 Z2];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on
    % face 4
    BaseX = [X2 X2 X2 X2];
    BaseY = [Y2 Y1 Y1 Y2];
    BaseZ = [Z1 Z1 Z2 Z2];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on
    % face 5
    BaseX = [X1 X2 X2 X1];
    BaseY = [Y2 Y2 Y2 Y2];
    BaseZ = [Z1 Z1 Z2 Z2];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on
    % face 6
    BaseX = [X2 X2 X1 X1];
    BaseY = [Y2 Y1 Y1 Y2];
    BaseZ = [Z2 Z2 Z2 Z2];
    patch(BaseX,BaseY,BaseZ,Color)
    hold on
end

% >>>> 圓柱體
for j = 1 : length(Color_cyl(:,1))
    r       = r_cyl(j); 
    color   = Color_cyl(j,:);
    X1      = start_point(j,:);
    X2      = end_point(j,:);
    
    Draw_Augment([], [], [], X1, X2, r, color, 'Cylinder');
end
end