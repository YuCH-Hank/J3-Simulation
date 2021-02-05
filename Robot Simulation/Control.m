function [Robot, Controller] = Control(Robot, Controller, Cmd, SamplingTime)

PosCmd = Cmd(1:6)';    VecCmd = Cmd(7:12)';
switch Controller.type
    
    case 'position'
        % >>>> Control law
        Robot.tor = Controller.position.Kv .* ( Controller.position.Kp .* ( PosCmd - Robot.pos ) - Robot.vec ) ;

        % >>>> Direct dynamic model 

        Robot.acc_old = Robot.acc ;
        Robot.vec_old = Robot.vec ;

        [ M , N ] = FullModel_MN( Robot.pos , Robot.vec ) ;

        Robot.acc =  M\( Robot.tor - N ) ;

        Robot.vec = Robot.vec + ( Robot.acc + Robot.acc_old ) .* ( SamplingTime / 2 ) ;
        Robot.pos = Robot.pos + ( Robot.vec + Robot.vec_old ) .* ( SamplingTime / 2 ) ;

    case 'velocity'
        
        % >>>> Control law
        Controller.velocity.error = [Controller.velocity.error, (VecCmd - Robot.vec)];
        sum_error = [sum(Controller.velocity.error(1,:));   sum(Controller.velocity.error(2,:));
                     sum(Controller.velocity.error(3,:));   sum(Controller.velocity.error(4,:));
                     sum(Controller.velocity.error(5,:));   sum(Controller.velocity.error(6,:))];
        Robot.tor = Controller.velocity.Kpv.*(Controller.velocity.Kiv .* sum_error .* SamplingTime - Robot.vec + Controller.velocity.Kfv .* VecCmd);

        % >>>> Direct dynamic model 

        Robot.acc_old = Robot.acc ;
        Robot.vec_old = Robot.vec ;

        [ M , N ] = FullModel_MN( Robot.pos , Robot.vec ) ;

        Robot.acc =  M\( Robot.tor - N ) ;

        Robot.vec = Robot.vec + ( Robot.acc + Robot.acc_old ) .* ( SamplingTime / 2 ) ;
        Robot.pos = Robot.pos + ( Robot.vec + Robot.vec_old ) .* ( SamplingTime / 2 ) ;
        
end

end