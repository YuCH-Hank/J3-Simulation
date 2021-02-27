function output = derivate(a, samp_t, degree)
output = [];
switch degree
    case 1
        da = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - a( ( 1 : ( end - 2 ) ) , : ) ) / ( 2 * samp_t ) ; zeros( 1 , 6 ) ] ;
        output = [a, da];
        
    case 2
        da = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - a( ( 1 : ( end - 2 ) ) , : ) ) / ( 2 * samp_t ) ; zeros( 1 , 6 ) ] ;
        dda = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - 2 * a( ( 2 : ( end - 1 ) ) , : ) + a( ( 1 : ( end - 2 ) ) , : ) ) / ( samp_t ^ 2 ) ; zeros( 1 , 6 ) ] ;  % 二階中央差分求加速度(頭尾補0)
        output = [a da, dda];
        
    case 3
        da = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - a( ( 1 : ( end - 2 ) ) , : ) ) / ( 2 * samp_t ) ; zeros( 1 , 6 ) ] ;
        dda = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - 2 * a( ( 2 : ( end - 1 ) ) , : ) + a( ( 1 : ( end - 2 ) ) , : ) ) / ( samp_t ^ 2 ) ; zeros( 1 , 6 ) ] ;  % 二階中央差分求加速度(頭尾補0)
        
        ddda = [ zeros( 2 , 6 ) ;  ...
            ( a( ( 5 : end ) , : ) - 2 * a( ( 4 : end - 1) , : ) + 2 * a( ( 2 : ( end - 3 ) ) , : )  - 1 * a( ( 1 : ( end - 4 ) ) , : ) ) /  (2 * samp_t ^ 3 ) ; ...
            zeros( 2 , 6 ) ] ;
        
        output = [a da, dda, ddda];
        
    case 4
        da = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - a( ( 1 : ( end - 2 ) ) , : ) ) / ( 2 * samp_t ) ; zeros( 1 , 6 ) ] ;
        dda = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - 2 * a( ( 2 : ( end - 1 ) ) , : ) + a( ( 1 : ( end - 2 ) ) , : ) ) / ( samp_t ^ 2 ) ; zeros( 1 , 6 ) ] ;  % 二階中央差分求加速度(頭尾補0)
        
        ddda = [ zeros( 2 , 6 ) ;  ...
            ( a( ( 5 : end ) , : ) - 2 * a( ( 4 : end - 1) , : ) + 2 * a( ( 2 : ( end - 3 ) ) , : )  - 1 * a( ( 1 : ( end - 4 ) ) , : ) ) /  (2 * samp_t ^ 3 ) ; ...
            zeros( 2 , 6 ) ] ;
        
        dddda = [ zeros( 2 , 6 ) ;  ...
            ( a( ( 5 : end ) , : ) - 4 * a( ( 4 : end - 1) , : ) + 6 * a( ( 3 : end - 2) , : ) - 4 * a( ( 2 : ( end - 3 ) ) , : )  + 1 * a( ( 1 : ( end - 4 ) ) , : ) ) /  ( samp_t ^ 4 ) ; ...
            zeros( 2 , 6 ) ] ;
        
        output = [a da, dda, ddda, dddda];
        
end
output = output;
end
