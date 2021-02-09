function [ JointCmd , Time ] = Scurve ( Initial , Final ,  acc_lim , a_avg , vec_lim , sampling_t)
% ================================================
% acc_lim           acceleration limitation (m/sec^2)
% acc_avg           average accleration = acc_lim * a_avg (m/sec^2)
% vec_lim           velocity limitation (m/sec)
% sampling_t        sampling time (s)
% ================================================

%% Calculate Displacement
Axis = length(Initial);

Displacement = Final - Initial;

[Max, Max_num] = max(abs(Displacement));

acc_avg = a_avg * acc_lim;



%% Calculate t
if (abs(Displacement(Max_num)) < abs(vec_lim^2 / acc_avg))
   vec_lim = sqrt(abs(Displacement(Max_num)) * (a_avg * acc_lim));
end

if Displacement(Max_num) < 0
    acc_lim = -acc_lim;
    vec_lim = -vec_lim;
end
acc_avg = a_avg * acc_lim;

Ta = vec_lim / acc_avg ;
Tb = 2 * vec_lim / acc_lim - Ta ;
Tc = ( Ta - Tb ) / 2 ;
Ts = ( Displacement(Max_num) - vec_lim * Ta ) / vec_lim;
    
t1 = Tc ;
t2 = Tc + Tb ;
t3 = Ta ;
t4 = Ta + Ts ;
t5 = Ta + Ts + Tc ;
t6 = Ta + Ts + Tc + Tb ;
t7 = Ta + Ts + Ta ;

%% Displacement Function
DisplacementFunction = ...
    1/6 * power(t1,3) + 1/6 * power(t2,3) + 1/3 * power(t3,3) +  1/6 * power(t4,3) - 1/6 * power(t5,3) - 1/6 * power(t6,3) + 1/6 * power(t7,3)...
    - 1/2 * (t1 * power(t3,2)) - 1/2 * (t7 * power(t1,2)) - 1/2 * t2 * power(t3,2) + (t1 * t3 * t7)...
    - 1/2 * (t7 * power(t2,2)) + (t2 * t3 * t7)...
    - 1/2 * (t7 * power(t3,2)) - 1/2 * (t7 * power(t4,2)) ...
    + 1/2 * (t4 * power(t7,2)) + 1/2 * (t7 * power(t5,2)) ...
    - 1/2 * (t5 * power(t7,2)) + 1/2 * (t7 * power(t6,2)) ...
    - 1/2 * (t6 * power(t7,2));

%% Jerk
K = zeros(1,Axis);
for i = 1:Axis
    K(i) = Displacement(i)/DisplacementFunction;
end

%% Calculate
n = 0;
for t = 0 : sampling_t : t7
    n = n + 1;
    for i = 1 : Axis
        if t1 >= t
            a(i,n) = K(i) * t;
            v(i,n) = K(i) * (1/2 * power(t,2));
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t,3));
            
        elseif t2 >= t && t > t1
            a(i,n) = K(i) * t1;
            v(i,n) = K(i) * (-1/2 * power(t1,2) + t1 * t);
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) - 1/2 * t * power(t1,2) + 1/2 * t1 * power(t,2));
            
        elseif t3 >= t && t > t2
            a(i,n) = -K(i) * t + K(i) * t1 + K(i) * t2 ;
            v(i,n) = K(i) * (-1/2 * power(t1,2) - 1/2 * power(t2,2) + t * t2 + t * t1 - 1/2 * power(t,2));
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) + 1/6 * power(t2,3) ...
                    - 1/2 * t * power(t1,2) - 1/2 * t * power(t2,2) + 1/2 * (t2 + t1) * power(t,2) - 1/6 * power(t,3));
            
        elseif t4 >= t && t > t3
            a(i,n) = 0;
            v(i,n) = K(i) * (-1/2 * power(t1,2) - 1/2 * power(t2,2) + t3 * t2 + t3 * t1 - 1/2 * power(t3,2));
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) + 1/6 * power(t2,3) +1/3 * power(t3,3) - 1/2 * t1 * power(t3,2) - 1/2 * t2 * power(t3,2)...
                    + t * (- 1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) + t1 * t3 + t2 * t3));
            
        elseif t5 >= t && t > t4
            a(i,n) = -K(i) * t + K(i) * t4;
            
            v(i,n) = K(i) * (-1/2 * power(t1,2) - 1/2 * power(t2,2) -1/2 * power(t3,2) -1/2 * power(t4,2) + t1 * t3 + t2 * t3 ...
                    + t * t4 - 1/2 * power(t,2));
                
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) + 1/6 * power(t2,3) + 1/3 * power(t3,3) + 1/6 * power(t4,3) -1/2 * t1 * power(t3,2) - 1/2 * t2 * power(t3,2)...
                    + t * (-1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) - 1/2 * power(t4,2) + t1 * t3 + t2 * t3)...
                    + 1/2 * t4 * power(t,2) - 1/6 * power(t,3));
            
        elseif t6 >= t && t > t5
            a(i,n) = -K(i) * t5 + K(i) * t4;
            
            v(i,n) = K(i) * (-1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) -1/2 * power(t4,2) + 1/2 * power(t5,2) + t1*t3 + t2*t3...
                    - t5 * t + t4 * t);
                
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) + 1/6 * power(t2,3) + 1/3 * power(t3,3) + 1/6 * power(t4,3) - 1/6 * power(t5,3)...
                    - 1/2 * t1 * power(t3,2) - 1/2 * t2 * power(t3,2) ...
                    + t * (-1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) - 1/2 * power(t4,2) + 1/2 * power(t5,2) + t1*t3 + t2 * t3)...
                    - 1/2 * (t5 - t4) * power(t,2));
                
			
        elseif t7 >= t && t > t6
            a(i,n) = K(i) * t - K(i) * t5 - K(i) * t6 + K(i) * t4;
            
            v(i,n) = K(i) * (-1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) -1/2 * power(t4,2) + 1/2 * power(t5,2) + 1/2 * power(t6,2) + t1* t3 + t2 * t3...
                    - t5 * t + t4 * t - t6 * t + 1/2 * power(t,2));
                
            s(i,n) = Initial(i) + K(i) * (1/6 * power(t1,3) + 1/6 * power(t2,3) + 1/3 * power(t3,3) + 1/6 * power(t4,3) - 1/6 * power(t5,3) - 1/6 * power(t6,3)...
                    -1/2 * t1 * power(t3,2) - 1/2 * t2 * power(t3,2)...
                    + t * (-1/2 * power(t1,2) - 1/2 * power(t2,2) - 1/2 * power(t3,2) - 1/2 * power(t4,2) + 1/2 * power(t5,2) + 1/2 * power(t6,2)...
                    + t1 * t3 + t2 * t3)...
                    + power(t,2) * 1/2 * (-t5 + t4 - t6) + 1/6 * power(t,3));
            
        end
        
    end
end

JointCmd = [s;v;a];
Time = 0 : sampling_t : t7;
