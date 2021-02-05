classdef Scurve_Single_Axis < handle
    properties
       % >>>> Initial Position, Final Position, Distance
       I_Pos, F_Pos, Distance
       
       % >>>> Limitation (Velocity, Accleration, Jerk, Snap
       Vmax, Amax, Jmax, Smax
       
       % >>>> Time
       Ts, Tj, Tv, Ta, T_sum
       
       % >>>> Property of Sigmoid
       a
       
       % >>>> Time
       Time, SamplingTime, Total_Time
       
       % >>>> Command
       Cmd                  = struct(   'Jerk',     [],...
                                        'Acc',      [],...
                                        'Vec',      [],...
                                        'Pos',      []);
                                    
       % >>>> Command Type
       Type
    end
    
    methods
        % >>>> Initial
        function obj = Scurve_Single_Axis()

        end
        
        % >>>> Set Input for Scurve
        function Input(obj, InitialPosition, FinalPosition, Vmax, Amax, Jmax, Smax, a, SamplingTime)
            obj.I_Pos       = InitialPosition;
            obj.F_Pos       = FinalPosition;
            obj.Distance    = FinalPosition - InitialPosition;
            obj.Vmax        = Vmax;
            obj.Amax        = Amax;
            obj.Jmax        = Jmax;
            obj.Smax        = Smax;
            obj.a           = a;
            obj.SamplingTime = SamplingTime;
        end
        
        function Clear(obj)
           obj.Distance = [];
           obj.Vmax = []; obj.Amax  = []; obj.Jmax  = []; obj.Smax  = [];         % Snap
           obj.Ts   = []; obj.Tj    = []; obj.Tv    = []; obj.Ta    = [];
           obj.a    = [];
           obj.Time = []; obj.SamplingTime = []; obj.Total_Time = [];
           obj.Cmd.Jerk = [];
           obj.Cmd	= struct(   'Jerk',     [],...
                                'Acc',      [],...
                                'Vec',      [],...
                                'Pos',      []);
           obj.Type = [];
        end
        
        % >>>> Calculate_Limitation
        function Calculate_Limitation(obj)
           Ts_d = (sqrt(3) * abs(obj.Distance) / (8 * obj.Smax)) ^ (1/4);
           Ts_v = (sqrt(3) * obj.Vmax / (2 * obj.Smax)) ^ (1/3);
           Ts_a = (sqrt(3) * obj.Amax / obj.Smax) ^ (1/2);
           Ts_j = sqrt(3) * obj.Jmax / obj.Smax;
           switch min([Ts_d, Ts_v, Ts_a, Ts_j])
               case Ts_d
                   obj.Ts = Ts_d;
                   obj.Tj = 0; obj.Tv = 0; obj.Ta = 0;
                   obj.Jmax = obj.Smax * obj.Ts / sqrt(3);
                   obj.Type = 1;
                   
               case Ts_v
                   obj.Ts = Ts_v;
                   obj.Tj = 0; obj.Ta = 0;
                   obj.Calculate_Tv;
                   obj.Jmax = obj.Smax * obj.Ts / sqrt(3);
                   obj.Type = 2;
                   
               case Ts_a
                   obj.Ts = Ts_a;
                   obj.Tj = 0;
                   obj.Jmax = obj.Smax * obj.Ts / sqrt(3);
                   obj.Type = 3;
                   obj.Calculate_Ta;
                   
               case Ts_j
                   obj.Ts = Ts_j;
                   obj.Calculate_Tj;
                   
           end
        end
        
        function Calculate_Tv(obj)
            obj.Tv = abs(obj.Distance) / obj.Vmax  - (4 * obj.Ts + 2 * obj.Tj + obj.Ta);
            
        end
        
        function Calculate_Ta(obj)
            Ta_d = (- (6 * obj.Ts + 3 * obj.Tj) + sqrt((2 * obj.Ts + obj.Tj)^2 + 4 * abs(obj.Distance) / obj.Amax))/2; 
            Ta_v = obj.Vmax / obj.Amax - 2*obj.Ts - obj.Tj;
            
            switch min([Ta_d, Ta_v])
                case Ta_d
                    obj.Ta = Ta_d;
                    obj.Tv = 0;
                    
                case Ta_v
                    obj.Ta = Ta_v;
                    obj.Calculate_Tv;
                    
            end
            
            if (obj.Type == 3)
               if (obj.Ta == Ta_d)
                   obj.Type = 3;
               end
               if (obj.Ta == Ta_v)
                   obj.Type = 4;
               end
            else
               if (obj.Ta == Ta_d)
                   obj.Type = 7;
               end
               if (obj.Ta == Ta_v)
                   obj.Type = 8;
               end
            end
        end
        
        function Calculate_Tj(obj)
            Tj_d = (obj.Ts ^3/27 + abs(obj.Distance)/(4 * obj.Jmax) + ...
                    sqrt( abs(obj.Distance) * obj.Ts ^ 3 / (54 * obj.Jmax) + obj.Distance^2/(16 * obj.Jmax^2) ) )^(1/3) + ...
                   (obj.Ts ^3/27 + abs(obj.Distance)/(4 * obj.Jmax) - ...
                    sqrt( abs(obj.Distance) * obj.Ts ^ 3 / (54 * obj.Jmax) + obj.Distance^2/(16 * obj.Jmax^2) ) )^(1/3) - ...
                    5 * obj.Ts / 3;
            Tj_v = - 3*obj.Ts/2 + sqrt(obj.Ts^2/4 + obj.Vmax/ obj.Jmax);
            Tj_a = obj.Amax/obj.Jmax - obj.Ts;
            
            switch min([Tj_d, Tj_v, Tj_a])
                case Tj_d
                    obj.Tj = Tj_d;
                    obj.Ta = 0;
                    obj.Tv = 0;
                    obj.Type = 5;
                    
                case Tj_v
                    obj.Tj = Tj_v;
                    obj.Ta = 0;
                    obj.Calculate_Tv;
                    obj.Type = 6;
                    
                case Tj_a
                    obj.Tj = Tj_a;
                    obj.Calculate_Ta;
            end
        end
        
        % >>>> Get Time Array
        function Calculate_Time(obj)
           obj.Time = [     0,...
                            1 * obj.Ts,...
                            1 * obj.Ts + 1 * obj.Tj,...
                            2 * obj.Ts + 1 * obj.Tj,...
                            2 * obj.Ts + 1 * obj.Tj + 1 * obj.Ta,...
                            3 * obj.Ts + 1 * obj.Tj + 1 * obj.Ta,...
                            3 * obj.Ts + 2 * obj.Tj + 1 * obj.Ta,...
                            4 * obj.Ts + 2 * obj.Tj + 1 * obj.Ta,...
                            4 * obj.Ts + 2 * obj.Tj + 1 * obj.Ta + 1 * obj.Tv,...
                            5 * obj.Ts + 2 * obj.Tj + 1 * obj.Ta + 1 * obj.Tv,...
                            5 * obj.Ts + 3 * obj.Tj + 1 * obj.Ta + 1 * obj.Tv,...
                            6 * obj.Ts + 3 * obj.Tj + 1 * obj.Ta + 1 * obj.Tv,...
                            6 * obj.Ts + 3 * obj.Tj + 2 * obj.Ta + 1 * obj.Tv,...
                            7 * obj.Ts + 3 * obj.Tj + 2 * obj.Ta + 1 * obj.Tv,...
                            7 * obj.Ts + 4 * obj.Tj + 2 * obj.Ta + 1 * obj.Tv,...
                            8 * obj.Ts + 4 * obj.Tj + 2 * obj.Ta + 1 * obj.Tv]; 
        end
        
        function tau = Calculate_tau(obj, t, a, b)
           tau = (t - obj.Time(a))/(obj.Time(b) - obj.Time(a));
        end

        function I_a = Calculate_Integral(obj, a)
            I_a = zeros(1,length(a));
            for i = 2:length(a)
                I_a(i) = I_a(i-1) + (a(i-1) + a(i))/2 * obj.SamplingTime;
            end
        end
        
        function Get_Time(obj)
            obj.Calculate_Time;
            obj.Total_Time = 0 : obj.SamplingTime : obj.Time(end);
        end
        
        function Get_Jerk(obj)
           if isempty(obj.Time) 
               obj.Calculate_Time;
           end
           
           for t = 0 : obj.SamplingTime : obj.Time(end)
               if (t >= obj.Time(1) && obj.Time(2) > t)
                   tau = obj.Calculate_tau(t, 1, 2);
                   if tau ~= 0
                        obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax / (1 + exp(- obj.a * (1/(1-tau) - 1/(tau))))];
                   else
                        obj.Cmd.Jerk = [obj.Cmd.Jerk 0];
                   end
               elseif t >= obj.Time(2) && obj.Time(3) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax];
                   
               elseif t >= obj.Time(3) && obj.Time(4) > t
                   tau = obj.Calculate_tau(t, 3, 4);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax / (1 + exp( obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(4) && obj.Time(5) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, 0];
                   
               elseif t >= obj.Time(5) && obj.Time(6) > t
                   tau = obj.Calculate_tau(t, 5, 6);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, -obj.Jmax / (1 + exp(- obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(6) && obj.Time(7) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, -obj.Jmax];
                   
               elseif t >= obj.Time(7) && obj.Time(8) > t
                   tau = obj.Calculate_tau(t, 7, 8);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, - obj.Jmax / (1 + exp( obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(8) && obj.Time(9) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, 0];
                   
               elseif t >= obj.Time(9) && obj.Time(10) > t
                   tau = obj.Calculate_tau(t, 9, 10);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, -obj.Jmax / (1 + exp(- obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(10) && obj.Time(11) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, -obj.Jmax];
                   
               elseif t >= obj.Time(11) && obj.Time(12) > t
                   tau = obj.Calculate_tau(t, 11, 12);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, - obj.Jmax / (1 + exp( obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(12) && obj.Time(13) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, 0];
                   
               elseif t >= obj.Time(13) && obj.Time(14) > t
                   tau = obj.Calculate_tau(t, 13, 14);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax / (1 + exp(- obj.a * (1/(1-tau) - 1/(tau))))];
                   
               elseif t >= obj.Time(14) && obj.Time(15) > t
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax];
                   
               elseif t >= obj.Time(15) && obj.Time(16) > t
                   tau = obj.Calculate_tau(t, 15, 16);
                   obj.Cmd.Jerk = [obj.Cmd.Jerk, obj.Jmax / (1 + exp( obj.a * (1/(1-tau) - 1/(tau))))];
               end
           end
        end

        function Get_Cmd(obj)
           if isempty (obj.Cmd.Jerk)
               obj.Get_Jerk;
           end
           obj.Cmd.Acc  = obj.Calculate_Integral(obj.Cmd.Jerk);
           obj.Cmd.Vec  = obj.Calculate_Integral(obj.Cmd.Acc);
           obj.Cmd.Pos  = obj.Calculate_Integral(obj.Cmd.Vec);
        end
        
        function Plot_Cmd(obj)
            if isempty(obj.Total_Time)
                obj.Get_Time;
            end
            if isempty(obj.Cmd.Pos)
                obj.Get_Cmd;
            end
            subplot(4,1,1);
            plot(obj.Total_Time, obj.Cmd.Pos,...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Distance,'r--');
            axis([0,obj.Total_Time(end), 0 , obj.Distance * 1.1]);
            title(sprintf('Type %d',obj.Type));
            xlabel('Time(s)');
            ylabel('Displacement(rad)');
            
            subplot(4,1,2);
            plot(obj.Total_Time, obj.Cmd.Vec,...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Vmax,'r--',...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Vmax,'r--');
            axis([0,obj.Total_Time(end), -obj.Vmax * 1.2 , obj.Vmax * 1.2]);
            xlabel('Time(s)');
            ylabel('Velocity(rad/s)');
            
            subplot(4,1,3);
            plot(obj.Total_Time, obj.Cmd.Acc,...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Amax,'r--',...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Amax,'r--');
            axis([0,obj.Total_Time(end), -obj.Amax * 1.2 , obj.Amax * 1.2]);
            xlabel('Time(s)');
            ylabel('Accleration(rad/s^2)');
            
            subplot(4,1,4);
            plot(obj.Total_Time, obj.Cmd.Jerk,...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Jmax,'r--',...
                obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Jmax,'r--');
            axis([0,obj.Total_Time(end), -obj.Jmax * 1.2 , obj.Jmax * 1.2]);
            xlabel('Time(s)');
            ylabel('Jerk(rad/s^3)');
            
        end
        
        % >>>> function for demo
        function Demo(obj, type)
            obj.a = sqrt(3)/2;
            obj.SamplingTime = 0.001;
            distance    = [4, 5, 4, 5, 10, 30, 10, 30];
            vmax        = [2.4933, 2.4933, 2.4933, 2.4933, 5, 5, 10, 3];
            amax        = [3.4907, 3.4907, 3, 3, 5, 5, 3, 1.5];
            jmax        = [10.4720, 10.4720, 10.4720, 10.4720, 3, 3, 1.5, 1.5];
            
            obj.Distance = distance(type);
            obj.Vmax = vmax(type);
            obj.Amax = amax(type);
            obj.Jmax = jmax(type);
            obj.Smax = obj.Jmax * 3;
        end
        
        function Plot_demo(obj)
            obj.Calculate_Limitation;
            obj.Plot_Cmd;
        end
    end
    
    
end