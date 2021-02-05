classdef Scurve_Jerk < handle
    properties
       % >>>> Initial Position, Final Position, Distance
       I_Pos, F_Pos, Distance
       
       % Limitation (Velocity, Accleration, Jerk, Snap)
       Vmax, Amax, Jmax, Smax, Initial
       
       % Time
       Ts, Tj, Tv, Ta, T_sum
       
       % Property of Sigmoid
       a
       
       % SamplingTime, Time Segment
       SamplingTime, Total_Time
       
       % Command
       Cmd                  = struct(   'Jerk',     [],...
                                        'Acc',      [],...
                                        'Vec',      [],...
                                        'Pos',      [],...
                                        'Time',     []);
       % Maximum of time
       Index, Time_max, Number_Command
       
       % Time sync
       Time_sync            = struct(   'Ts',       [],...
                                        'Ta',       [],...
                                        'Tv',       [],...
                                        'Tj',       [],...
                                        'Scaling',  []);
    end
    
    methods
        
        % >>>> Initial
        function obj = Scurve_Jerk()

        end
        
        % >>>> Given Settings
        function Input(obj, InitialPosition, FinalPosition, Vmax, Amax, Jmax, Smax, a, SamplingTime)
            obj.I_Pos       = InitialPosition;
            obj.F_Pos       = FinalPosition;
            obj.Distance    = FinalPosition - InitialPosition;
            obj.Initial     = struct('Vmax' , Vmax, 'Amax'  , Amax, 'Jmax'  , Jmax, 'Smax'  , Smax);
            obj.Vmax        = Vmax;
            obj.Amax        = Amax;
            obj.Jmax        = Jmax;
            obj.Smax        = Smax;
            obj.a           = a;
            obj.SamplingTime = SamplingTime;
        end
        
        % >>>> Clear Data
        function Clear(obj)
           obj.Distance = [];
           obj.Vmax = []; obj.Amax  = []; obj.Jmax  = []; obj.Smax  = [];
           obj.Ts   = []; obj.Tj    = []; obj.Tv    = []; obj.Ta    = [];
           obj.a = [];
           obj.SamplingTime = []; obj.Total_Time = [];
           obj.Cmd  = struct(   'Jerk',     [],...
                                'Acc',      [],...
                                'Vec',      [],...
                                'Pos',      [],...
                                'Time',     []);
          obj.Time_sync = struct(   'Ts',       [],...
                                    'Ta',       [],...
                                    'Tv',       [],...
                                    'Tj',       [],...
                                    'Scaling',  []);
           obj.Index = []; obj.Time_max = [];
        end
        
        % >>>> MultiAxis Calculate Limitation
        function MultiAxis_Time_optimial_Cmd(obj)
            % >>>> Check Each Axis Limitation
            for i = 1 : length(obj.Distance)
                [ts, tj, ta, tv, jmax] = SingleAxis(obj.Distance(i), obj.Vmax(i), obj.Amax(i), obj.Jmax(i), obj.Smax(i));
                obj.Ts = [obj.Ts ts];
                obj.Tj = [obj.Tj tj];
                obj.Ta = [obj.Ta ta];
                obj.Tv = [obj.Tv tv];
                obj.Jmax(i) = jmax;
            end
            
            % >>>> Get Total Time
            obj.T_sum = obj.Ts .*8 + obj.Tj * 4 + obj.Ta * 2 + obj.Tv ;
            [obj.Time_max, obj.Index] = max(obj.T_sum);
            
            % >>>> TimeSync and Optimal
            [obj.Jmax, obj.Ts, obj.Tj, obj.Ta, obj.Tv] = ...
                TimeSync(obj.T_sum, obj.Time_max, obj.Ts, obj.Tj, obj.Ta, obj.Tv, 0.0001, obj.Distance, obj.Vmax, obj.Amax, obj.Jmax, obj.Smax);
                      
            % >>>> New Total Time
            obj.T_sum = obj.Ts .*8 + obj.Tj * 4 + obj.Ta * 2 + obj.Tv ;
            
            % >>>> Command Time
            obj.Cmd.Time = 0 : obj.SamplingTime : obj.Time_max ;
            obj.Number_Command = length(obj.Cmd.Time);
            
            % >>>> Get Command
            for n = 1:length(obj.Distance)
                if (obj.Distance(n) ~= 0)
                    [J, A, V, P] = GenerateCmd( real(obj.Ts(n)), real(obj.Tj(n)), real(obj.Ta(n)), real(obj.Tv(n)),...
                        real(obj.Jmax(n)) * sign(obj.Distance(n)), obj.a, obj.SamplingTime);
                    if (length(P) < obj.Number_Command)
                        P = [P, ones(1,obj.Number_Command - length(P)) * P(end)];
                        V = [V, ones(1,obj.Number_Command - length(V)) * V(end)];
                        A = [A, ones(1,obj.Number_Command - length(A)) * A(end)];
                        J = [J, ones(1,obj.Number_Command - length(J)) * J(end)];
                        obj.Cmd.Pos  = [obj.Cmd.Pos;  P + ones(1, obj.Number_Command) .* obj.I_Pos(n)];
                        obj.Cmd.Vec  = [obj.Cmd.Vec;  V];
                        obj.Cmd.Acc  = [obj.Cmd.Acc;  A];
                        obj.Cmd.Jerk = [obj.Cmd.Jerk; J];
                    else
                        obj.Cmd.Pos  = [obj.Cmd.Pos;  P(1:obj.Number_Command) + ones(1, obj.Number_Command) .* obj.I_Pos(n)];
                        obj.Cmd.Vec  = [obj.Cmd.Vec;  V(1:obj.Number_Command)];
                        obj.Cmd.Acc  = [obj.Cmd.Acc;  A(1:obj.Number_Command)];
                        obj.Cmd.Jerk = [obj.Cmd.Jerk; J(1:obj.Number_Command)];
                    end
                else
                    obj.Cmd.Pos  = [obj.Cmd.Pos;  ones(1, obj.Number_Command) .* obj.I_Pos(n)];
                    obj.Cmd.Vec  = [obj.Cmd.Vec;  zeros(1,obj.Number_Command)];
                    obj.Cmd.Acc  = [obj.Cmd.Acc;  zeros(1,obj.Number_Command)];
                    obj.Cmd.Jerk = [obj.Cmd.Jerk; zeros(1,obj.Number_Command)];
                end
            end
            
            % >>>> Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            % >>>> SingleAxis Limitation
            function [Ts, Tj, Ta, Tv, jmax] = SingleAxis(Distance, Vmax, Amax, Jmax, Smax)
                Ts_d = (sqrt(3) * abs(Distance) / (8 * Smax)) ^ (1/4);
                Ts_v = (sqrt(3) * Vmax / (2 * Smax)) ^ (1/3);
                Ts_a = (sqrt(3) * Amax / Smax) ^ (1/2);
                Ts_j = sqrt(3) * Jmax / Smax;
                switch min([Ts_d, Ts_v, Ts_a, Ts_j])
                    case Ts_d
                        Ts = Ts_d;
                        Tj = 0; Tv = 0; Ta = 0;
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_v
                        Ts = Ts_v;
                        Tj = 0; Ta = 0;
                        Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_a
                        Ts = Ts_a;
                        Tj = 0;
                        jmax = Smax * Ts / sqrt(3);
                        [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                        
                    case Ts_j
                        Ts = Ts_j;
                        jmax = Jmax;
                        [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts);
                        
                end
                
                % >>>> Calculate Tv
                function Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta)
                    Tv = abs(Distance) / Vmax - (4 * Ts + 2 * Tj + Ta);
                    
                end
                
                % >>>> Calculate Ta
                function [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj)
                    Ta_d = (- (6 * Ts + 3 * Tj) + sqrt((2 * Ts + Tj)^2 + 4 * abs(Distance) / Amax))/2;
                    Ta_v = Vmax / Amax - 2 * Ts - Tj;
                    
                    switch min([Ta_d, Ta_v])
                        case Ta_d
                            Ta = Ta_d; Tv = 0;
                            
                        case Ta_v
                            Ta = Ta_v;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                    end
                end
                
                % >>>> Calculate Tj
                function [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts)
                    Tj_d = (Ts ^3/27 + abs(Distance)/(4 * Jmax) + ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) + ...
                           (Ts ^3/27 + abs(Distance)/(4 * Jmax) - ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) - ...
                           5 * Ts / 3;
                    Tj_v = - 3 * Ts/2 + sqrt(Ts^2/4 + Vmax/ Jmax);
                    Tj_a = Amax/Jmax - Ts;
                    
                    switch min([Tj_d, Tj_v, Tj_a])
                        case Tj_d
                            Tj = Tj_d; Ta = 0; Tv = 0;
                            
                        case Tj_v
                            Tj = Tj_v; Ta = 0;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                        case Tj_a
                            Tj = Tj_a;
                            [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                    end
                end
                
            end
            
            % >>>> Generate Command
            function [Jcmd, Acmd, Vcmd, Pcmd] = GenerateCmd(Ts, Tj, Ta, Tv, Jmax, a, SamplingTime)
                time = Calculate_Time(Ts, Tj, Ta, Tv);
                Jcmd = Get_Jerk(time, Jmax, a, SamplingTime);
                Acmd = Calculate_Integral(Jcmd, SamplingTime);
                Vcmd = Calculate_Integral(Acmd, SamplingTime);
                Pcmd = Calculate_Integral(Vcmd, SamplingTime);
                
                % >>>> Time Segment
                function Time = Calculate_Time(Ts, Tj, Ta, Tv)
                    Time = [     0,...
                            1 * Ts,...
                            1 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 4 * Tj + 2 * Ta + 1 * Tv,...
                            8 * Ts + 4 * Tj + 2 * Ta + 1 * Tv];
                end
                
                % >>>> Calculate tau
                function tau = Calculate_tau( t, a, b, Time)
                    tau = (t - Time(a))/(Time(b) - Time(a));
                end
                
                % >>>> Jerk Command
                function Jerk = Get_Jerk(Time, Jmax, a, SamplingTime)
                    Jerk = [];
                    for t = 0 : SamplingTime : Time(end)
                        if (t >= Time(1) && Time(2) > t)
                            tau = Calculate_tau(t, 1, 2, Time);
                            if tau ~= 0
                                Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            else
                                Jerk = [Jerk 0];
                            end
                        elseif t >= Time(2) && Time(3) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(3) && Time(4) > t
                            tau = Calculate_tau(t, 3, 4, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(4) && Time(5) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(5) && Time(6) > t
                            tau = Calculate_tau(t, 5, 6, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(6) && Time(7) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(7) && Time(8) > t
                            tau = Calculate_tau(t, 7, 8, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(8) && Time(9) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(9) && Time(10) > t
                            tau = Calculate_tau(t, 9, 10, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(10) && Time(11) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(11) && Time(12) > t
                            tau = Calculate_tau(t, 11, 12, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(12) && Time(13) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(13) && Time(14) > t
                            tau = Calculate_tau(t, 13, 14, Time);
                            Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(14) && Time(15) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(15) && Time(16) > t
                            tau = Calculate_tau(t, 15, 16, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                        end
                    end
                end
                
                % >>>> Numerical Integration
                function I_a = Calculate_Integral( a, SamplingTime)
                    I_a = zeros(1,length(a));
                    for j = 2:length(a)
                        I_a(j) = I_a(j-1) + (a(j-1) + a(j))/2 * SamplingTime;
                    end
                end
            end
            
            % >>>> Min jerk Synchronization
            function [Jsync, Ts_, Tj_, Ta_, Tv_] = ...
                TimeSync(Sum_Time_k, Time_sync, Ts, Tj, Ta, Tv, error, Distance_k, Vmax_k, Amax_k, Jmax_k, Smax_k)
                for k = 1:length(Distance_k)
                    if (Sum_Time_k(k) == Time_sync || Distance_k(k) == 0)
                       Jsync(k) = Jmax_k(k);
                       Ts_(k) = Ts(k);          Tj_(k) = Tj(k);
                       Ta_(k) = Ta(k);          Tv_(k) = Tv(k);
                       
                    else
                        J_LB = 0; J_UB = Jmax_k(k); T_UB = Sum_Time_k(k); T_LB = 0;
                        
                        while(abs(Sum_Time_k(k) - Time_sync) > error)
                            Jmax_k_ = (J_LB + J_UB)/2;
                            [Ts_(k), Tj_(k), Ta_(k), Tv_(k), Jmax_k(k)] = ...
                                SingleAxis(Distance_k(k), Vmax_k(k), Amax_k(k), Jmax_k_, Smax_k(k));
                            
                            Sum_Time_k(k) = 8 * Ts_(k) + 4 * Tj_(k) + 2 * Ta_(k) + 1 * Tv_(k);
                            
                            if (J_LB ~= 0)
                                if Sum_Time_k(k) < Time_sync
                                    Jmax_k_ = (T_LB * Jmax_k_ - Sum_Time_k(k) * J_LB + (J_LB - Jmax_k_) * Time_sync)/(T_LB - Sum_Time_k(k));
                                else
                                    Jmax_k_ = (Sum_Time_k(k) * J_UB - T_UB * Jmax_k_ + (Jmax_k_ - J_UB) * Time_sync)/(Sum_Time_k(k) - T_UB);
                                end
                                [Ts_(k), Tj_(k), Ta_(k), Tv_(k), Jmax_k(k)] = ...
                                    SingleAxis(Distance_k(k), Vmax_k(k), Amax_k(k), Jmax_k_, Smax_k(k));
                                Sum_Time_k(k) = 8 * Ts_(k) + 4 * Tj_(k) + 2 * Ta_(k) + 1 * Tv_(k);
                            end
                            
                            if (Sum_Time_k(k) > Time_sync)
                                J_LB = Jmax_k_; T_LB = Sum_Time_k(k);
                            else
                                J_UB = Jmax_k_; T_UB = Sum_Time_k(k);
                            end
                        end
                        
                        Jsync(k) = Jmax_k_;
                        Ts_(k) = Ts_(k);          Tj_(k) = Tj_(k);
                        Ta_(k) = Ta_(k);          Tv_(k) = Tv_(k);
                    end
                end
                
                % >>>> Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                % >>>> SingleAxis Limitation
                function [Ts, Tj, Ta, Tv, jmax] = SingleAxis(Distance, Vmax, Amax, Jmax, Smax)
                    Ts_d = (sqrt(3) * abs(Distance) / (8 * Smax)) ^ (1/4);
                    Ts_v = (sqrt(3) * Vmax / (2 * Smax)) ^ (1/3);
                    Ts_a = (sqrt(3) * Amax / Smax) ^ (1/2);
                    Ts_j = sqrt(3) * Jmax / Smax;
                    switch min([Ts_d, Ts_v, Ts_a, Ts_j])
                        case Ts_d
                            Ts = Ts_d;
                            Tj = 0; Tv = 0; Ta = 0;
                            jmax = Smax * Ts / sqrt(3);

                        case Ts_v
                            Ts = Ts_v;
                            Tj = 0; Ta = 0;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            jmax = Smax * Ts / sqrt(3);

                        case Ts_a
                            Ts = Ts_a;
                            Tj = 0;
                            jmax = Smax * Ts / sqrt(3);
                            [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);

                        case Ts_j
                            Ts = Ts_j;
                            jmax = Jmax;
                            [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts);

                    end

                    % >>>> Calculate Tv
                    function Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta)
                        Tv = abs(Distance) / Vmax - (4 * Ts + 2 * Tj + Ta);

                    end

                    % >>>> Calculate Ta
                    function [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj)
                        Ta_d = (- (6 * Ts + 3 * Tj) + sqrt((2 * Ts + Tj)^2 + 4 * abs(Distance) / Amax))/2;
                        Ta_v = Vmax / Amax - 2 * Ts - Tj;

                        switch min([Ta_d, Ta_v])
                            case Ta_d
                                Ta = Ta_d; Tv = 0;

                            case Ta_v
                                Ta = Ta_v;
                                Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);

                        end
                    end

                    % >>>> Calculate Tj
                    function [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts)
                        Tj_d = (Ts ^3/27 + abs(Distance)/(4 * Jmax) + ...
                                sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) + ...
                               (Ts ^3/27 + abs(Distance)/(4 * Jmax) - ...
                                sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) - ...
                               5 * Ts / 3;
                        Tj_v = - 3 * Ts/2 + sqrt(Ts^2/4 + Vmax/ Jmax);
                        Tj_a = Amax/Jmax - Ts;

                        switch min([Tj_d, Tj_v, Tj_a])
                            case Tj_d
                                Tj = Tj_d; Ta = 0; Tv = 0;

                            case Tj_v
                                Tj = Tj_v; Ta = 0;
                                Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);

                            case Tj_a
                                Tj = Tj_a;
                                [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                        end
                    end

                end
                            
            end
            
        end
        
        % >>>> MultiAxis Time Sync
        function MultiAxis_Time_Sync(obj)
            % >>>> Check Each Axis Limitation
            for i = 1 : length(obj.Distance)
                [ts, tj, ta, tv, jmax] = SingleAxis(obj.Distance(i), obj.Vmax(i), obj.Amax(i), obj.Jmax(i), obj.Smax(i));
                obj.Ts = [obj.Ts ts];
                obj.Tj = [obj.Tj tj];
                obj.Ta = [obj.Ta ta];
                obj.Tv = [obj.Tv tv];
                obj.Jmax(i) = jmax;
            end
            
            % >>>> Get Total Time
            obj.T_sum = obj.Ts .*8 + obj.Tj * 4 + obj.Ta * 2 + obj.Tv ;
            [obj.Time_max, obj.Index] = max(obj.T_sum);
            obj.Time_sync.Scaling = obj.Time_max ./ obj.T_sum;
            obj.Cmd.Time = 0 : obj.SamplingTime : obj.Time_max ;
            
            % >>>> Generate Command
            obj.Ts = obj.Ts .* obj.Time_sync.Scaling;
            obj.Ta = obj.Ta .* obj.Time_sync.Scaling;
            obj.Tv = obj.Tv .* obj.Time_sync.Scaling;
            obj.Tj = obj.Tj .* obj.Time_sync.Scaling;
            obj.Jmax = obj.Jmax ./ obj.Time_sync.Scaling .^3 .* sign(obj.Distance);
            
            for n = 1:length(obj.Distance)
                if (obj.Distance(n)~= 0)
                    [obj.Cmd.Jerk(n,:), obj.Cmd.Acc(n,:), obj.Cmd.Vec(n,:), obj.Cmd.Pos(n,:)] = ...
                        GenerateCmd( real(obj.Ts(n)), real(obj.Tj(n)), real(obj.Ta(n)), real(obj.Tv(n)),...
                        real(obj.Jmax(n)), obj.a, obj.SamplingTime);

                    obj.Cmd.Pos(n,:) = obj.Cmd.Pos(n,:) + ones(1,length(obj.Cmd.Vec(n,:))) .* obj.I_Pos(n);
                else
                    obj.Cmd.Jerk(n,:) = zeros(1,length(obj.Cmd.Time));
                    obj.Cmd.Acc(n,:) = zeros(1,length(obj.Cmd.Time));
                    obj.Cmd.Vec(n,:) = zeros(1,length(obj.Cmd.Time));
                    obj.Cmd.Pos(n,:) = ones(1,length(obj.Cmd.Time)) .* obj.I_Pos(n);
                end
            end
            
            % >>>> Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            % >>>> SingleAxis Limitation
            function [Ts, Tj, Ta, Tv, jmax] = SingleAxis(Distance, Vmax, Amax, Jmax, Smax)
                Ts_d = (sqrt(3) * abs(Distance) / (8 * Smax)) ^ (1/4);
                Ts_v = (sqrt(3) * Vmax / (2 * Smax)) ^ (1/3);
                Ts_a = (sqrt(3) * Amax / Smax) ^ (1/2);
                Ts_j = sqrt(3) * Jmax / Smax;
                switch min([Ts_d, Ts_v, Ts_a, Ts_j])
                    case Ts_d
                        Ts = Ts_d;
                        Tj = 0; Tv = 0; Ta = 0;
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_v
                        Ts = Ts_v;
                        Tj = 0; Ta = 0;
                        Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_a
                        Ts = Ts_a;
                        Tj = 0;
                        jmax = Smax * Ts / sqrt(3);
                        [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                        
                    case Ts_j
                        Ts = Ts_j;
                        jmax = Jmax;
                        [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts);
                        
                end
                
                % >>>> Calculate Tv
                function Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta)
                    Tv = abs(Distance) / Vmax - (4 * Ts + 2 * Tj + Ta);
                    
                end
                
                % >>>> Calculate Ta
                function [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj)
                    Ta_d = (- (6 * Ts + 3 * Tj) + sqrt((2 * Ts + Tj)^2 + 4 * abs(Distance) / Amax))/2;
                    Ta_v = Vmax / Amax - 2 * Ts - Tj;
                    
                    switch min([Ta_d, Ta_v])
                        case Ta_d
                            Ta = Ta_d; Tv = 0;
                            
                        case Ta_v
                            Ta = Ta_v;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                    end
                end
                
                % >>>> Calculate Tj
                function [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts)
                    Tj_d = (Ts ^3/27 + abs(Distance)/(4 * Jmax) + ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) + ...
                           (Ts ^3/27 + abs(Distance)/(4 * Jmax) - ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) - ...
                           5 * Ts / 3;
                    Tj_v = - 3 * Ts/2 + sqrt(Ts^2/4 + Vmax/ Jmax);
                    Tj_a = Amax/Jmax - Ts;
                    
                    switch min([Tj_d, Tj_v, Tj_a])
                        case Tj_d
                            Tj = Tj_d; Ta = 0; Tv = 0;
                            
                        case Tj_v
                            Tj = Tj_v; Ta = 0;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                        case Tj_a
                            Tj = Tj_a;
                            [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                    end
                end
                
            end
            
            % >>>> Generate Command
            function [Jcmd, Acmd, Vcmd, Pcmd] = GenerateCmd(Ts, Tj, Ta, Tv, Jmax, a, SamplingTime)
                time = Calculate_Time(Ts, Tj, Ta, Tv);
                Jcmd = Get_Jerk(time, Jmax, a, SamplingTime);
                Acmd = Calculate_Integral(Jcmd, SamplingTime);
                Vcmd = Calculate_Integral(Acmd, SamplingTime);
                Pcmd = Calculate_Integral(Vcmd, SamplingTime);
                
                % >>>> Time Segment
                function Time = Calculate_Time(Ts, Tj, Ta, Tv)
                    Time = [     0,...
                            1 * Ts,...
                            1 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 4 * Tj + 2 * Ta + 1 * Tv,...
                            8 * Ts + 4 * Tj + 2 * Ta + 1 * Tv];
                end
                
                % >>>> Calculate tau
                function tau = Calculate_tau( t, a, b, Time)
                    tau = (t - Time(a))/(Time(b) - Time(a));
                end
                
                % >>>> Jerk Command
                function Jerk = Get_Jerk(Time, Jmax, a, SamplingTime)
                    Jerk = [];
                    for t = 0 : SamplingTime : Time(end)
                        if (t >= Time(1) && Time(2) > t)
                            tau = Calculate_tau(t, 1, 2, Time);
                            if tau ~= 0
                                Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            else
                                Jerk = [Jerk 0];
                            end
                        elseif t >= Time(2) && Time(3) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(3) && Time(4) > t
                            tau = Calculate_tau(t, 3, 4, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(4) && Time(5) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(5) && Time(6) > t
                            tau = Calculate_tau(t, 5, 6, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(6) && Time(7) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(7) && Time(8) > t
                            tau = Calculate_tau(t, 7, 8, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(8) && Time(9) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(9) && Time(10) > t
                            tau = Calculate_tau(t, 9, 10, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(10) && Time(11) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(11) && Time(12) > t
                            tau = Calculate_tau(t, 11, 12, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(12) && Time(13) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(13) && Time(14) > t
                            tau = Calculate_tau(t, 13, 14, Time);
                            Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(14) && Time(15) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(15) && Time(16) > t
                            tau = Calculate_tau(t, 15, 16, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                        end
                    end
                end
                
                % >>>> Numerical Integration
                function I_a = Calculate_Integral( a, SamplingTime)
                    I_a = zeros(1,length(a));
                    for j = 2:length(a)
                        I_a(j) = I_a(j-1) + (a(j-1) + a(j))/2 * SamplingTime;
                    end
                end
            end
            
            % >>>> Numerical Integration
            function I_a = Calculate_Integral( a, SamplingTime)
                I_a = zeros(1,length(a));
                for j = 2:length(a)
                    I_a(j) = I_a(j-1) + (a(j-1) + a(j))/2 * SamplingTime;
                end
            end
        end
        
        % >>>> MultiAxis Plot
        function MultiAxis_plot(obj)
            figure ('name','pos')
            for i = 1:6
                subplot(2,3,i);
                plot(obj.Cmd.Time, obj.Cmd.Pos(i,:),...
                    obj.Cmd.Time, ones(1,obj.Number_Command) .* obj.I_Pos(i),'r--',...
                    obj.Cmd.Time, ones(1,obj.Number_Command) .* obj.F_Pos(i),'r--');
                title('Pos');
                xlabel('Time(s)');
                ylabel('Displacement(rad)');
                legend(sprintf('J%d',i));
                hold on;
            end
            hold off;
            figure ('name','vec')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Vec(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Vmax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Vmax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Vec(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Vmax(i) * 1.2 , obj.Vmax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Velocity(rad/s)');
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
            hold off;
            figure ('name','acc')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Acc(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Amax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Amax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Acc(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Amax(i) * 1.2 , obj.Amax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Accleration(rad/s^2)');
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
            hold off;
            figure ('name','jerk')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Jerk(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Jmax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Jmax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Jerk(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Jmax(i) * 1.2 , obj.Jmax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Jerk(rad/s^3)');
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
        end
        
        % >>>> MultiAxis Plot
        function MultiAxis_plot_converge(obj)
            figure ('name','pos')
            for i = 1:6
                plot(obj.Cmd.Time, obj.Cmd.Pos(i,:));
                title('Pos');
                xlabel('Time(s)');
                ylabel('Displacement(rad)');
                grid on;
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
            hold off;
            figure ('name','vec')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Vec(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Vmax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Vmax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Vec(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Vmax(i) * 1.2 , obj.Vmax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Velocity(rad/s)');
                grid on;
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
            hold off;
            figure ('name','acc')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Acc(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Amax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Amax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Acc(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Amax(i) * 1.2 , obj.Amax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Accleration(rad/s^2)');
                grid on;
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
            hold off;
            figure ('name','jerk')
            for i = 1:6
%                 plot(obj.Total_Time, obj.Cmd.Jerk(i,:),...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* obj.Jmax(i),'r--',...
%                     obj.Total_Time, ones(1,length(obj.Total_Time)) .* -obj.Jmax(i),'r--');
                plot(obj.Cmd.Time, obj.Cmd.Jerk(i,:));
%                 axis([0,obj.Total_Time(end), -obj.Jmax(i) * 1.2 , obj.Jmax(i) * 1.2]);
                xlabel('Time(s)');
                ylabel('Jerk(rad/s^3)');
                grid on;
                hold on;
            end
            legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6');
        end
        
        % >>>> SingleAxis Limitation
        function SingleAxis_Limitation(obj)
            [obj.Ts, obj.Tj, obj.Ta, obj.Tv, obj.Jmax] = SingleAxis(obj.Distance, obj.Vmax, obj.Amax, obj.Smax);
            [obj.Cmd.Jerk, obj.Cmd.Acc, obj.Cmd.Vec, obj.Cmd.Pos] = GenerateCmd(...
                obj.Ts, obj.Tj, obj.Ta, obj.Tv, obj.Jmax, obj.a, obj.SamplingTime);
            
            obj.Cmd.Time = 0 : obj.SamplingTime : (8 * obj.Ts + 4 * obj.Tj + 2 * obj.Ta + obj.Tv) ;
            
            % >>>> Functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            % >>>> SingleAxis Limitation
            function [Ts, Tj, Ta, Tv, jmax] = SingleAxis(Distance, Vmax, Amax, Jmax, Smax)
                Ts_d = (sqrt(3) * abs(Distance) / (8 * Smax)) ^ (1/4);
                Ts_v = (sqrt(3) * Vmax / (2 * Smax)) ^ (1/3);
                Ts_a = (sqrt(3) * Amax / Smax) ^ (1/2);
                Ts_j = sqrt(3) * Jmax / Smax;
                switch min([Ts_d, Ts_v, Ts_a, Ts_j])
                    case Ts_d
                        Ts = Ts_d;
                        Tj = 0; Tv = 0; Ta = 0;
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_v
                        Ts = Ts_v;
                        Tj = 0; Ta = 0;
                        Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                        jmax = Smax * Ts / sqrt(3);
                        
                    case Ts_a
                        Ts = Ts_a;
                        Tj = 0;
                        jmax = Smax * Ts / sqrt(3);
                        [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                        
                    case Ts_j
                        Ts = Ts_j;
                        jmax = Jmax;
                        [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts);
                        
                end
                
                % >>>> Calculate Tv
                function Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta)
                    Tv = abs(Distance) / Vmax - (4 * Ts + 2 * Tj + Ta);
                    
                end
                
                % >>>> Calculate Ta
                function [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj)
                    Ta_d = (- (6 * Ts + 3 * Tj) + sqrt((2 * Ts + Tj)^2 + 4 * abs(Distance) / Amax))/2;
                    Ta_v = Vmax / Amax - 2 * Ts - Tj;
                    
                    switch min([Ta_d, Ta_v])
                        case Ta_d
                            Ta = Ta_d; Tv = 0;
                            
                        case Ta_v
                            Ta = Ta_v;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                    end
                end
                
                % >>>> Calculate Tj
                function [Tj, Ta, Tv] = Calculate_Tj(Distance, Jmax, Amax, Vmax, Ts)
                    Tj_d = (Ts ^3/27 + abs(Distance)/(4 * Jmax) + ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) + ...
                           (Ts ^3/27 + abs(Distance)/(4 * Jmax) - ...
                            sqrt( abs(Distance) * Ts ^ 3 / (54 * Jmax) + Distance^2/(16 * Jmax^2) ) )^(1/3) - ...
                           5 * Ts / 3;
                    Tj_v = - 3 * Ts/2 + sqrt(Ts^2/4 + Vmax/ Jmax);
                    Tj_a = Amax/Jmax - Ts;
                    
                    switch min([Tj_d, Tj_v, Tj_a])
                        case Tj_d
                            Tj = Tj_d; Ta = 0; Tv = 0;
                            
                        case Tj_v
                            Tj = Tj_v; Ta = 0;
                            Tv = Calculate_Tv(Distance, Vmax, Ts, Tj, Ta);
                            
                        case Tj_a
                            Tj = Tj_a;
                            [Ta,Tv] = Calculate_Ta(Distance, Amax, Vmax, Ts, Tj);
                    end
                end
                
            end
            
            % >>>> Generate Command
            function [Jcmd, Acmd, Vcmd, Pcmd] = GenerateCmd(Ts, Tj, Ta, Tv, Jmax, a, SamplingTime)
                time = Calculate_Time(Ts, Tj, Ta, Tv);
                Jcmd = Get_Jerk(time, Jmax, a, SamplingTime);
                Acmd = Calculate_Integral(Jcmd, SamplingTime);
                Vcmd = Calculate_Integral(Acmd, SamplingTime);
                Pcmd = Calculate_Integral(Vcmd, SamplingTime);
                
                % >>>> Time Segment
                function Time = Calculate_Time(Ts, Tj, Ta, Tv)
                    Time = [     0,...
                            1 * Ts,...
                            1 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj,...
                            2 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 1 * Tj + 1 * Ta,...
                            3 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta,...
                            4 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 2 * Tj + 1 * Ta + 1 * Tv,...
                            5 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 1 * Ta + 1 * Tv,...
                            6 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 3 * Tj + 2 * Ta + 1 * Tv,...
                            7 * Ts + 4 * Tj + 2 * Ta + 1 * Tv,...
                            8 * Ts + 4 * Tj + 2 * Ta + 1 * Tv];
                end
                
                % >>>> Calculate tau
                function tau = Calculate_tau( t, a, b, Time)
                    tau = (t - Time(a))/(Time(b) - Time(a));
                end
                
                % >>>> Jerk Command
                function Jerk = Get_Jerk(Time, Jmax, a, SamplingTime)
                    Jerk = [];
                    for t = 0 : SamplingTime : Time(end)
                        if (t >= Time(1) && Time(2) > t)
                            tau = Calculate_tau(t, 1, 2, Time);
                            if tau ~= 0
                                Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            else
                                Jerk = [Jerk 0];
                            end
                        elseif t >= Time(2) && Time(3) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(3) && Time(4) > t
                            tau = Calculate_tau(t, 3, 4, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(4) && Time(5) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(5) && Time(6) > t
                            tau = Calculate_tau(t, 5, 6, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(6) && Time(7) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(7) && Time(8) > t
                            tau = Calculate_tau(t, 7, 8, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(8) && Time(9) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(9) && Time(10) > t
                            tau = Calculate_tau(t, 9, 10, Time);
                            Jerk = [Jerk, -Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(10) && Time(11) > t
                            Jerk = [Jerk, -Jmax];
                            
                        elseif t >= Time(11) && Time(12) > t
                            tau = Calculate_tau(t, 11, 12, Time);
                            Jerk = [Jerk, - Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(12) && Time(13) > t
                            Jerk = [Jerk, 0];
                            
                        elseif t >= Time(13) && Time(14) > t
                            tau = Calculate_tau(t, 13, 14, Time);
                            Jerk = [Jerk, Jmax / (1 + exp(- a * (1/(1-tau) - 1/(tau))))];
                            
                        elseif t >= Time(14) && Time(15) > t
                            Jerk = [Jerk, Jmax];
                            
                        elseif t >= Time(15) && Time(16) > t
                            tau = Calculate_tau(t, 15, 16, Time);
                            Jerk = [Jerk, Jmax / (1 + exp( a * (1/(1-tau) - 1/(tau))))];
                        end
                    end
                end
                
                % >>>> Numerical Integration
                function I_a = Calculate_Integral( a, SamplingTime)
                    I_a = zeros(1,length(a));
                    for j = 2:length(a)
                        I_a(j) = I_a(j-1) + (a(j-1) + a(j))/2 * SamplingTime;
                    end
                end
            end
            
        end
        
        % >>>> Get Command
        function [Command, Time] = Get_Command(obj)
            Command = [];
            Time = obj.Cmd.Time;
            Command = [Command; obj.Cmd.Pos];
            Command = [Command; obj.Cmd.Vec];
            Command = [Command; obj.Cmd.Acc];
            Command = [Command; obj.Cmd.Jerk];
        end
        
    end
    
    
end