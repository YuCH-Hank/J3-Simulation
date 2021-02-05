%%  畫圖 畫物件 輸入: center(x, y), Width(a, b, c), Color
function Draw_Box(center, Width, Color)

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