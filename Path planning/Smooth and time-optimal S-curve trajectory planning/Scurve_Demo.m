close all; clc; clear;
%% %%%%%%%%%%%% Scurve MultiAxis with Jerk Limitation
% Date: 2021/01/22
% =================================================================================
%% Limitation

% >>>> ITRI_parameter
[ITRI_parameter, DH_table, SamplingTime] = ITRI_Parameter;

% >>>> ITRI_constraint
ITRI_Limitation = ITRI_Constraint( ITRI_parameter.GearRatio );

%% Scurve Single Axis Demo
% >>>> Initial
% Scurve = Scurve_Single_Axis();
% 
% for i = 1:8
%     Scurve.Demo(i);
%     figure('name', sprintf('Figure %d' , i));
%     Scurve.Plot_demo;
%     Scurve.Clear;
% end
% Scurve.Clear;

%% Scurve Multi Axis
% InitialPosition = [0 10   0 0 30 2];
% FinalPosition   = [20 1 -30 10 10  5];
% Scurve = Scurve_Jerk();
% Scurve.Input(   InitialPosition,            FinalPosition, ...
%                 ITRI_Limitation.Joint.Vel,  ITRI_Limitation.Joint.Acc, ...
%                 ITRI_Limitation.Joint.Jerk, ITRI_Limitation.Joint.Jerk * 10, ...
%                 sqrt(3)/2, SamplingTime);

InitialPosition = [0 1.5  0 0 3 1];
FinalPosition   = [1 2 3 1 2  1];
Vel_Limitation  = ITRI_Limitation.Joint.Vel;
Acc_Limitation  = ITRI_Limitation.Joint.Acc;
Jerk_Limitation = Acc_Limitation .* 10;
Snap_Limitation = Jerk_Limitation .*10;


% >>>> Time sync
Scurve = Scurve_Jerk();
Scurve.Input(   InitialPosition,            FinalPosition, ...
                Vel_Limitation,  Acc_Limitation, ...
                Jerk_Limitation, Snap_Limitation, ...
                sqrt(3)/2, SamplingTime);
Scurve.MultiAxis_Time_Sync;
Scurve.MultiAxis_plot_converge;

% >>>> Minimum Jerk
Scurve = Scurve_Jerk();
Scurve.Input(   InitialPosition,            FinalPosition, ...
                Vel_Limitation,  Acc_Limitation, ...
                Jerk_Limitation, Snap_Limitation, ...
                sqrt(3)/2, SamplingTime);
Scurve.MultiAxis_Time_optimial_Cmd;
Scurve.MultiAxis_plot_converge;
