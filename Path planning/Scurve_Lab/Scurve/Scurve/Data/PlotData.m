clc ; clear ; close all ;

%% 讀取實驗資料

MeasuredData = load( 'Scurve_Minimize_Jerk.txt' ) ;
%MeasuredData = load( 'Scurve_TimeSync.txt' ) ;
%MeasuredData = load( 'Scurve_MultiAxisLimitation.txt' ) ;
% MeasuredData = load( 'Scurve_Original.txt' ) ;


PosCmd = MeasuredData( : , 1 : 6 ) ;
VelCmd = MeasuredData( : , 7 : 12 ) ; 
AccCmd = MeasuredData( : , 13 : 18 ) ; 
%JerkCmd = MeasuredData( : , 19 : 24 ) ; 


%% 時間

n = 6 ;
st = 0.001 ;
tf = size( MeasuredData , 1 ) * st ;
Time = st : st : tf ;


%% 繪製結果圖

figure( 1 )

for i = 1 : n
    
    plot( Time , PosCmd( : , i ) , '-'  ) ;
    title( [ 'Pos ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Pos (rad)', 'FontSize' , 10 ) ;
    grid on ;
    hold on;
end
hold off;
figure( 2 )

for i = 1 : n
    
    plot( Time , VelCmd( : , i ) ) ;
    title( [ 'Vel ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Vel (rad/s)', 'FontSize' , 10 ) ;
    grid on ;
    hold on;
    
end

figure( 3 )

for i = 1 : n
    
    plot( Time , AccCmd( : , i )  ) ;
    title( [ 'AccCmd ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
    xlabel( 'Time (sec)') ; ylabel( 'Acc (rad/s^2)', 'FontSize' , 10 ) ;
    grid on ;
    hold on;
    
end

% figure( 4 )
% 
% for i = 1 : n
%     
%     plot( Time , JerkCmd( : , i )  ) ;
%     title( [ 'JerkCmd ' , num2str(i) ] , 'FontWeight' , 'bold' , 'FontSize' , 12 ) ;
%     xlabel( 'Time (sec)') ; ylabel( 'Jerk (rad/s^3)', 'FontSize' , 10 ) ;
%     grid on ;
%     hold on;
%     
% end


