classdef Class_Controller < handle
    properties
        type
        position = struct(  'Kp'        , [ 250 ; 250 ; 200 ; 250 ; 150 ; 350 ] ,...
                            'Kv'        , [ 100 ; 100 ; 100 ; 40  ; 30  ; 5 ]);
        velocity = struct(  'Kiv'       , [ 100; 150; 150; 70; 70; 50 ] ,...
                            'Kfv'       , [ 1 ; 1 ; 1   ; 1 ; 1 ; 1 ],...
                            'Kpv'       , [ 150 ; 150 ; 100 ; 50 ; 100 ; 50],...
                            'error'     , []);
    end
    
    methods
        function this = Class_Controller(type)
            this.type = type;
        end
        
        function Robot = Control_Law(this, Cmd, Robot, SamplingTime)
            PosCmd = Cmd(1:6)';    VecCmd = Cmd(7:12)';
            switch this.type
                case 'position'
                    % >>>> Control law
                    Robot.tor = this.position.Kv .* ( this.position.Kp .* ( PosCmd - Robot.pos ) - Robot.vec ) ;
                    
                    % >>>> Direct dynamic model
                    
                    Robot.acc_old = Robot.acc ;
                    Robot.vec_old = Robot.vec ;
                    
                    [ M , N ] = FullModel_MN( Robot.pos , Robot.vec ) ;
                    
                    Robot.acc =  M\( Robot.tor - N ) ;
                    
                    Robot.vec = Robot.vec + ( Robot.acc + Robot.acc_old ) .* ( SamplingTime / 2 ) ;
                    Robot.pos = Robot.pos + ( Robot.vec + Robot.vec_old ) .* ( SamplingTime / 2 ) ;
                    
                case 'velocity'
                    
                    % >>>> Control law
                    this.velocity.error = [this.velocity.error, (VecCmd - Robot.vec)];
                    sum_error = [   sum(this.velocity.error(1,:));   sum(this.velocity.error(2,:));
                                    sum(this.velocity.error(3,:));   sum(this.velocity.error(4,:));
                                    sum(this.velocity.error(5,:));   sum(this.velocity.error(6,:))];
                    Robot.tor = this.velocity.Kpv.*(this.velocity.Kiv .* sum_error .* SamplingTime - Robot.vec + this.velocity.Kfv .* VecCmd);
                    
                    % >>>> Direct dynamic model
                    
                    Robot.acc_old = Robot.acc ;
                    Robot.vec_old = Robot.vec ;
                    
                    [ M , N ] = FullModel_MN( Robot.pos , Robot.vec ) ;
                    
                    Robot.acc =  M\( Robot.tor - N ) ;
                    
                    Robot.vec = Robot.vec + ( Robot.acc + Robot.acc_old ) .* ( SamplingTime / 2 ) ;
                    Robot.pos = Robot.pos + ( Robot.vec + Robot.vec_old ) .* ( SamplingTime / 2 ) ;
            end
            
        end
        
        function Collection(this)
            switch this.type
               case('position')
                   fprintf('\nPosition Controller\n');
                   fprintf('\tKp : ');fprintf('%.1f\t',this.position.Kp');fprintf('\n');
                   fprintf('\tKv : ');fprintf('%.1f\t',this.position.Kv');fprintf('\n');
               case('velocity')
                   fprintf('\tVelocity Controller\n');
                   fprintf('\tKiv : ');fprintf('%.1f\t',this.velocity.Kiv');fprintf('\n');
                   fprintf('\tKfv : ');fprintf('%.1f\t',this.velocity.Kfv');fprintf('\n');
                   fprintf('\tKpv : ');fprintf('%.1f\t',this.velocity.Kpv');fprintf('\n');
            end
            
        end
    end
    
end