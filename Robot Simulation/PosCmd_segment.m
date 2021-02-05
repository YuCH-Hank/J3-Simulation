function [Poscmd, Endflag] = PosCmd_segment(InitialPose, FinalPose, max_step_number, segment_size)

Displacement = FinalPose - InitialPose;
Poscmd = [];
% >>>> Check
steps = round(abs( Displacement ./ segment_size ) + ones(1, length(Displacement)));

if max(steps) > max_step_number
    printf('Over limitaiton');
    Endflag = true;
else
    for i = 1:length(Displacement)
        Poscmd = [Poscmd; linspace(InitialPose(i), FinalPose(i), max(steps))];
    end
    Poscmd = Poscmd';
    Endflag = false;
end

end