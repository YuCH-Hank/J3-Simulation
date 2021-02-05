for i = 1:6
    subplot(2,3,i)
   plot(Record.Joint.JointCmd(i,:),'r');
   hold on;
   plot(Record.Joint.JointRecord(:,i),'b');
   hold off;
end

Record.Joint.JointRecord = lowpass(Record.Joint.JointRecord, 100, SamplingTime^-1);
samp_t = SamplingTime;
a = Record.Joint.JointRecord;
dJoint = [ zeros( 1 , 6 ) ; ( a( ( 3 : end ) , : ) - a( ( 1 : ( end - 2 ) ) , : ) ) / ( 2 * samp_t ) ; zeros( 1 , 6 ) ] ;

figure(2);
len = length(Record.Joint.VelCmd(i,:));
for i = 1:6
   subplot(2,3,i)
   plot(Record.Joint.VelCmd(i,1:len -50),'r');
   hold on;
   plot(dJoint(1:len -50,i),'b');
   hold off;
end