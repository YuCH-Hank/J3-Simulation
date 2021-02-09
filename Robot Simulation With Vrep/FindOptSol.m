function OptimalSol = FindOptSol(InvSol , NowJointPos )

Sol = InvSol;
IsSolution = ones(8,1) ;
MinDistance = 0 ;
buffer = 0;
numofsol = 0;

for i = 1 : 8
    
    for j = 1 : 6

        if( abs(Sol(i,j)) >= 3.145 )        
            IsSolution(i) = 0;
            break;
        end
        
    end
    
    if( IsSolution(i) == 1 )
        numofsol = numofsol + 1 ;
    end
        
end


if (numofsol == 0)

    disp('此點在機械手臂工作範圍外，請關閉程式');
    OptimalSol(1) = 0.0;
    OptimalSol(2) = 0.0;
    OptimalSol(3) = 0.0;
    OptimalSol(4) = 0.0;
    OptimalSol(5) = 0.0;
    OptimalSol(6) = 0.0;
    
else
    
    OptimalSolutionNumber = 0;
    for i = 1 : 8
    
        if( IsSolution(i) == 1 )
            
            % 以輸出關節角度與目前角度最接近之解為目標
            distance = 0 ;
            for j = 1 : 6
            
                buffer = InvSol(i,j) - NowJointPos(j);
                distance = distance + buffer*buffer;
                buffer = 0;                
            end
            
            if( OptimalSolutionNumber == 0 )
                
                Mindistance = distance;
                OptimalSolutionNumber = i;
                
            else
                
                if( distance < Mindistance )
                    
                    Mindistance = distance;
                    OptimalSolutionNumber = i;
                    
                end
            end
        end
    end
    
   
    OptimalSolutionNumber;
    OptimalSol(1) = Sol(OptimalSolutionNumber,1);
    OptimalSol(2) = Sol(OptimalSolutionNumber,2);
    OptimalSol(3) = Sol(OptimalSolutionNumber,3);
    OptimalSol(4) = Sol(OptimalSolutionNumber,4);
    OptimalSol(5) = Sol(OptimalSolutionNumber,5);
    OptimalSol(6) = Sol(OptimalSolutionNumber,6);

end
    
    
end