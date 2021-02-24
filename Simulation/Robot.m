classdef Robot < handle
   properties
       l                % Length(m)
       c                % Centroid Distance(m)
       m                % Mass(kg)
       I                % Moment of Interia
   end
   
   methods
       function this = Robot()
           
       end
       
       function Initial(this)
           this.l = [0.4, 0.4];                 % Length(m)
           this.c = [0.2, 0.2];                 % Centroid distance(m)
           this.m = [2, 2];                     % Mass(kg)
           this.I = [0.0267, 0.0267];           % Moment of inertia(kg.m^2)
       end
       
       function Joint_Cmd = Inverse_Kinemetic(this, C_Cmd)
           x = C_Cmd(:,1);            y = C_Cmd(:,2);
           
           theta2 = acos((x.^2 + y.^2-this.l(1)^2-this.l(2)^2)./(2*this.l(1)*this.l(2)));
           
           for  i = 1  : length(x)
               delta = (this.l(1) + this.l(2)*cos(theta2(i)))^2 + this.l(2)^2*sin(theta2(i))^2;
               belta = ((this.l(1) + this.l(2)*cos(theta2(i)))*x(i) + this.l(2)*sin(theta2(i))*y(i))/delta;
               alpha = ((this.l(1) + this.l(2)*cos(theta2(i)))*y(i) - this.l(2)*sin(theta2(i))*x(i))/delta;
               
               if (belta >= 0 )
                   theta(i) = atan(alpha/belta);
                   
               elseif (alpha >=0 )
                   theta(i) = atan(alpha/belta) + pi;
                   
               else
                   theta(i) = atan(alpha/belta) - pi;
               end
           end
           Joint_Cmd = [theta; theta2];
       end
   end
    
end