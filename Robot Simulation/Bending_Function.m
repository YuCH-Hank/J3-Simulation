function N = Bending_Function(knot, SamplingTime, p)
%% %%%%%%%%%%%%%%%%%%%%%%%
% p             degree = k-1    級數
% k             order  = p+1    階數
% knot          knot vector
% SamplingTime  SamplingTime
%% %%%%%%%%%%%%%%%%%%%%%%%
% >>>> parameter 
k = p + 1;      % order

% >>>> n = number of control point - 1
n = length(knot)-1;

% >>>> Initial of Blending function
N = zeros(n, k, max(knot) / SamplingTime + 1);

%%
for j = 1:k    % N(,j)
    for i = 1:n    % N(i,)
        % >>>> Initial >>>>>>>>>>>>>>>>>>>>>
        % >>>> N(:,1) 
        if j == 1   
            % >>>> 前 p 個 = 0
            if i <= p
                start =   1; end_ = p + 1;
                
            % >>>> 後 p 個 = 1
            elseif i > length(knot) - (p + 1)
                start = length(knot) - (p + 1); end_ = length(knot);
            
            % >>>> 中間項
            else
                start =   i; end_ = i+1;
                
            end
                
            for u = 0 : SamplingTime : max(knot)
                % >>>> 後 p 項, u = knot(end)
                if  i >= length(knot) - (p + 1) && u == knot(end)
                    N(i, j, round(u / SamplingTime + 1)) = 1;
                
                % >>>> if knot(start) < u < knot(end) -> N(i,1) = 1
                elseif  knot(start) <= u && u < knot(end_)
                    N(i, j, round(u/SamplingTime + 1)) = 1;
                    
                end
            end
            
        % >>>> Order more than 1 >>>>>>>>>>>>>>>>>>>>>
        else
            % >>>> 多一階 少一項
            if i + j > n + 1
                break;
            end
            
            
            for u = 0 : SamplingTime : max(knot)
                if knot(i+j-1) == knot(i)
                    par1 = 0;
                    
                else
                    par1 = (u - knot(i)) / (knot(i+j-1)-knot(i));
                    
                end
                
                if knot(i+j) == knot(i+1)
                    par2 = 0;
                    
                else
                    par2 = (knot(i+j) - u) / (knot(i+j)-knot(i+1));
                    
                end
                
                N(i,j, round( u/SamplingTime + 1)) = ...
                    par1 * N(   i, j-1, round( u/SamplingTime + 1))+...
                    par2 * N( i+1, j-1, round( u/SamplingTime + 1));
                
            end
        end
    end
end

end